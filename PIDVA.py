from hub import motion_sensor, port
import motor, motor_pair, utime
import math, motor
def start(port1,port2,leftspeed,rightspeed):
    motor.run(port1,-10*leftspeed)
    motor.run(port2,10*rightspeed)
def stop(port1,port2):
    motor.stop(port1)
    motor.stop(port2)
def reset(port1,port2):
    motor.reset_relative_position(port1,0)
    motor.reset_relative_position(port2,0)
def quaternion_to_yaw(qi):
    # Extract quaternion components
    w, x, y, z = qi

    # Convert quaternion to Euler angles
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)

    # Convert from radians to degrees
    yaw_degrees = math.degrees(yaw)*-1

    return yaw_degrees
def resetyaw():
    quaternion = motion_sensor.quaternion()
    return quaternion
def yaw(qq,rq=(0,0,0,0)):
    yaw = quaternion_to_yaw(qq) - quaternion_to_yaw(rq)
    if yaw>180:
        yaw = -180 - (180 - yaw)
    if yaw<-180:
        yaw = 180 + (180 + yaw)
    return yaw
# Create a motion sensor object
motion_sensor = motion_sensor
motors = motor_pair
distanceTraveled=0
ogyaw = resetyaw()
motor.reset_relative_position(port.A,0)
motor.reset_relative_position(port.B,0)
#function
def gyro(desiredDegree, distance, deadband, maxspeed=80, minspeed=10, distDeadband=0.5, ka=20, kv=10, rapidness=0, kp=0.7, ki=0.000125, kd=3, wheelLockLeft=False, wheelLockRight=False,dime=False,turnspeed=8,turndiff=3): #maybe switch kp to 1 to 3. turndiff maybe should be 3
    motion = motion_sensor
    ticks=utime.ticks_ms()
    motion.reset_yaw(0)
    integral=0
    currente=ticks
    paste=0
    error2=0
    erroryaw=0
    error=0
    currenterrordist = distance
    ryaw = resetyaw()
    error1=desiredDegree - quaternion_to_yaw(ryaw)
    while True:
        quaternion = motion_sensor.quaternion()
        yawangle = yaw(quaternion,ryaw)
        ticks=utime.ticks_ms()
        #Proportional
        p = kp*abs(desiredDegree-yawangle)
        #Derivitave
        paste=currente
        currente=ticks
        time=utime.ticks_diff(currente,paste)
        erroryaw=desiredDegree - yawangle
        error2=error1
        error1=erroryaw
        totalerror=abs(error1)-abs(error2)
        d=0
        if time>0:
            d = (totalerror/time)*kd
        #Distance checking
        a=0
        v=0
        speed=0
        if distance!=0:
            distanceTraveled1 = (motor.relative_position(port.A)/360)*-17.89
            distanceTraveled2 = (motor.relative_position(port.B)/360)*17.89
            distanceTraveled = (distanceTraveled1+distanceTraveled2)/2
            error = distance - distanceTraveled
            #Acceleration
            a = 100-abs(distance/2-distanceTraveled)*ka
            pasterrordist=currenterrordist
            currenterrordist=error
            deltaerrordist = abs(currenterrordist)-abs(pasterrordist)
            try:
                v = (deltaerrordist/time)*kv
            except:
                v = 0
            if distance>0:
                speed=int(a)+int(v)+rapidness
                if speed<minspeed:
                    speed=minspeed
                if speed>maxspeed:
                    speed=maxspeed
            if distance<0:
                speed=0-int(a)+int(v)-rapidness
                if speed>-1*minspeed:
                    speed=-1*minspeed
                if speed<-1*maxspeed:
                    speed=-1*maxspeed
            #stopping (for distance)
            if distance-distDeadband<distanceTraveled<distance+distDeadband:
                stop(port.A,port.B)
                reset(port.A,port.B)
                break
        #stopping (for turning)
        else:
            if desiredDegree-deadband<yawangle<desiredDegree+deadband:
                stop(port.A,port.B)
                reset(port.A,port.B)
                break
        #Integral
        integral = integral+erroryaw
        i = abs(integral) * ki
        #Gyro correction
        if yawangle>desiredDegree+deadband: #original deadband was 5
            if wheelLockRight == True:
                start(port.A,port.B,0,turnspeed+int(p)+int(i)+int(d))
            elif wheelLockLeft==True:
                start(port.A,port.B,0-turnspeed-int(p)-int(i)-int(d),0)
            elif dime==True:
                start(port.A,port.B,0-turnspeed-int(p)-int(i)-int(d),turnspeed+int(p)+int(i)+int(d))
            else:
                start(port.A,port.B,turnspeed-turndiff,turnspeed+int(p)+int(i)+int(d))
        elif yawangle<desiredDegree-deadband:
            if wheelLockRight == True:
                start(port.A,port.B,turnspeed+int(p)+int(i)+int(d),0)
            elif wheelLockLeft==True:
                start(port.A,port.B,0,0-turnspeed-int(p)-int(i)-int(d))
            elif dime==True:
                start(port.A,port.B,turnspeed+int(p)+int(i)+int(d),0-turnspeed-int(p)-int(i)-int(d))
            else:
                start(port.A,port.B,turnspeed+int(p)+int(i)+int(d),turnspeed-turndiff)
        else:
            if distance!=0:
                start(port.A,port.B,speed,speed)
            else:
                stop(port.A,port.B)
                reset(port.A,port.B)
                break
#function call w/ args
#ka should be around 2-5 (depends on distance higher for short distances low for long distances(like 20+))
#pid straight should be kp = 1, kd = 0, turnspeed = 40, turndiff = 10, deadband = 5. Turn should be default deadband = 0. Turn pid values should be default. same for turnspeed and turndiff.
gyro(90,0,0.1)
