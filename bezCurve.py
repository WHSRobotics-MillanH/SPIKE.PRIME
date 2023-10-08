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
#function
def gyro(desiredX, desiredY, currentX,currentY, desiredDegree, distance, deadband, ka=0, kv=0, rapidness=50, kp=0.7, ki=0.000125, kd=3, wheelLockLeft=False, wheelLockRight=False,dime=False,turnspeed=8,turndiff=3): #maybe switch kp to 1 to 3. turndiff maybe should be 3
    motion = motion_sensor
    global realtimeX
    global realtimeY
    realtimeY=0
    realtimeX=0
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
    errorX=desiredX-currentX
    errorY=desiredY-currentY
    distnosqrt=abs(errorX)**2+abs(errorY)**2
    dist=sqrt(abs(distnosqrt))
    degrees=(asin(errorX/dist))*(180/pi)
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
        v=0
        a=0
        if time>0:
            d = (totalerror/time)*kd
        #Distance checking
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
            #Realtime x
            #kind of reverses the sine formula in order to get the current X in realtime
            realtimeX = sin(math.radians(degrees))*distanceTraveled
            realtimeY = cos(math.radians(degrees))*distanceTraveled
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
    return realtimeX, realtimeY
def coordSysStraight(desiredX, desiredY, currentX,currentY):
    #finding the errors on the x and y (how far of it is)
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
    gyro(desiredX, desiredY, currentX,currentY, degrees,0,1)
    gyro(desiredX, desiredY, currentX,currentY, degrees,dist,0.5)

def coordSysCurved(desiredX, desiredY, smoothness, turnamt):
    currentX=0
    currentY=0
    errorXactual = desiredX-currentX
    errorYactual = desiredY-currentY
    desiredXactual = desiredX
    desiredX=currentX
    count=0
    pastdegreesum=0

    while True:
        #finding the errors on the x and y (how far of it is)
        errorX=desiredX-currentX
        errorY=desiredY-currentY
        #using the pythagorean theorem (distance algorithm) to find the hypotenuse which is equal to the distance needed to travel.
        distnosqrt=abs(errorX)**2+abs(errorY)**2
        dist=sqrt(abs(distnosqrt))
        #finding the degrees using sine and then converting from rad to degrees
        #(i know sine doesnt return in degrees but its relativeley close and i dont wanna input an entire hashmap and the sine function doesn't convert sine to degrees)
        degrees=(asin(errorX/dist))*(180/pi)-pastdegreesum
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
        if abs(desiredX)>=abs(desiredXactual):
            desiredX=desiredXactual
        if abs(currentY)>=abs(desiredY):
            degrees=0
        #pid call statements (args order: desired degrees, distance to travel, deadband (ignore the deadband for now i was experiementing the old one was 1 or 2))
        gyro(desiredX, desiredY, currentX,currentY,degrees,0,1)
        gyro(desiredX, desiredY, currentX,currentY,degrees,smoothness,0.5)
        #this series of if statement supdates the realtimeX, currentX, and currentY in order to keep it consistent with the curve. the comments in the if statements 
        if errorXactual>0:
            if errorYactual>0 and not(abs(currentY)>=abs(desiredY)):
                if count>0:
                    currentX+=abs(realtimeX)
                desiredX+=turnamt
                currentY+=abs(realtimeY)
            elif errorYactual>0 and not(abs(currentY)>=abs(desiredY)):
                if count>0:
                    #we increase currentX by the mount we move horizontally.
                    currentX+=abs(realtimeX)
                #we increase our curve desired towards our actual desired X
                desiredX+=turnamt
                #current y adds the amount we move up
                currentY-=abs(realtimeY)
            elif abs(currentY)>=abs(desiredY):
                currentX+=smoothness
        if errorXactual<0:
            if errorYactual>0:
                if count>0:
                    currentX-=abs(realtimeX)
                desiredX-=turnamt
                currentY+=abs(realtimeY)
            else:
                if count>0:
                    currentX-=abs(realtimeX)
                desiredX-=turnamt
                currentY-=abs(realtimeY)
        # this is to make sure we dont increase desired X too much but i dont think this is needed. its just a safety measure
        
        count+=1
        pastdegreesum+=degrees
        if currentX-1<desiredXactual<currentX+1 and currentY-1<desiredY<currentY+1:
            break

coordSysCurved(10,4,1,1)
