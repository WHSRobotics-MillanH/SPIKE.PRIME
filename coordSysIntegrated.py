from hub import motion_sensor, port
import motor, motor_pair, utime
import math, time, motor
from math import sin,cos,asin,acos,pi,sqrt
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
realcurrentX=0
realcurrentY=0
#function
def coordSys(desiredX, desiredY, ka=7, kv=10, rapidness=0, kp=0.7, ki=0.000125, kd=3, wheelLockLeft=False, wheelLockRight=False,dime=False,turnspeed=8,turndiff=3): #maybe switch kp to 1 to 3. turndiff maybe should be 3
    currentX = realcurrentX
    currentY = realcurrentY
    global realcurrentX
    global realcurrentY
    realcurrentX = desiredX
    realcurrentY = desiredY
    errorX=desiredX-currentX
    errorY=desiredY-currentY
    #using the pythagorean theorem (distance algorithm) to find the hypotenuse which is equal to the distance needed to travel.
    distnosqrt=abs(errorX)**2+abs(errorY)**2
    dist=sqrt(abs(distnosqrt))
    #finding the degrees using sine and then converting from rad to degrees
    #(i know sine doesnt return in degrees but its relativeley close and i dont wanna input an entire hashmap and the sine function doesn't convert sine to degrees)
    degrees=(asin(errorX/dist))*(180/pi)
    #before it was not asin but just sin
    #this redefines the degrees depending on whether errorY is positive or negative and whether or not x or y are 0.
    if errorY<0:
        if errorX<0:
            degrees = -180-degrees
        if errorX>0:
            degrees = 180-degrees
    if errorX==0:
        if desiredX>currentX:
            degrees = 90
        if desiredX<currentX:
            degrees = -90
    if errorY==0:
        if desiredY>currentY:
            degrees = 0
        if desiredY<currentY:
            degrees = 180
    #pid call statements (args order: desired degrees, distance to travel, deadband (ignore the deadband for now i was experiementing the old one was 1 or 2))
    desiredDegree=degrees
    distance=dist
    deadband=0.5
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
    #realtime tracking
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
        if time>0:
            d = (totalerror/time)*kd
        #Distance checking
        if distance!=0:
            distanceTraveled1 = (motor.relative_position(port.A)/360)*-17.89
            distanceTraveled2 = (motor.relative_position(port.B)/360)*17.89
            global distanceTraveled
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
            #stopping (for distance)
            if round(distanceTraveled,0)==distance:
                stop(port.A,port.B)
                reset(port.A,port.B)
                break
        #stopping (for turning)
        else:
            if yawangle==desiredDegree:
                stop(port.A,port.B)
                reset(port.A,port.B)
                break
        #Integral
        integral = abs(integral+erroryaw)
        i = integral * ki
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
            if distance>0:
                start(port.A,port.B,int(a)+rapidness+int(v),int(a)+rapidness+int(v))
            elif distance<0:
                start(port.A,port.B,0+int(a)-int(v)-rapidness,0+int(a)-int(v)-rapidness)
                print(a)
            else:
                stop(port.A,port.B)
                reset(port.A,port.B)
                break
        #uses sine and cosine to find the length of the adjacent and opposite sides (aka x and y). the degrees would stay consistent thru the triangle for each so thats why it theoretically works.
        #im pretty sure you know how to find the adjacent and opposite sides but just in case i do sin(degrees) = opposite/hypotenuse. hypotenuse is distancetraveled and then i just do algebra
        #to find the opposite aka the x value
        realtimeX=distanceTraveled*sin(degrees)
        realtimeY=distanceTraveled*cos(degrees)
        #this is for realtime errors
        realtimeErrorX = desiredX-realtimeX
        realtimeErrorY = desiredY-realtimeY
        # this is for testing purposes
        print(realtimeX,",",realtimeY)
        print(distanceTraveled)
        #this is to break out of the while loop so it can go on to future statemetents
