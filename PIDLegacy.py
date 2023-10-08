from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import utime

hub = PrimeHub()

#hub.light_matrix.write('PLS WORK')

#defins motors + resets yaw angle (imp for gyro so it doesn't have to be zero angle initially)
motors = MotorPair('A','E') 
mb = Motor('E') 
mf = Motor('A')
yaw = hub.motion_sensor
distanceTraveled=0
mb.set_degrees_counted(0)
mf.set_degrees_counted(0)
#function
def gyro(desiredDegree, distance, deadband, ka=2, rapidness=15, kp=0.7, ki=0.000125, kd=3, wheelLockLeft=False, wheelLockRight=False,dime=False,turnspeed=8,turndiff=3): #maybe switch kp to 1 to 3. turndiff maybe should be 3
    yaw = hub.motion_sensor
    integral=0
    currente=0
    paste=0
    error1=0
    error2=0
    erroryaw=0
    error=0
    yaw.reset_yaw_angle()
    while True:
        ticks=utime.ticks_ms()
        #Proportional
        p = kp*abs(desiredDegree-yaw.get_yaw_angle())
        #Derivitave
        paste=currente
        currente=ticks
        time=utime.ticks_diff(currente,paste)
        erroryaw=desiredDegree - yaw.get_yaw_angle()
        error2=error1
        error1=erroryaw
        totalerror=abs(error1)-abs(error2)
        if time>0:
            d = (totalerror/time)*kd
        #Distance checking
        if distance!=0:
            distanceTraveled1 = (mb.get_degrees_counted()/360)*17.89
            distanceTraveled2 = (mf.get_degrees_counted()/360)*-17.89
            distanceTraveled = (distanceTraveled1+distanceTraveled2)/2
            error = distance - distanceTraveled
            #Acceleration
            a = int(error)*ka
            #stopping (for distance)
            if round(distanceTraveled,0)==distance:
                mb.set_degrees_counted(0)
                mf.set_degrees_counted(0)
                motors.stop()
                break
                return 0
        #stopping (for turning)
        else:
            if yaw.get_yaw_angle()==desiredDegree:
                motors.stop()
                mb.set_degrees_counted(0)
                mf.set_degrees_counted(0)
                break
                return 0
        #Integral
        integral = abs(integral+erroryaw)
        i = integral * ki
        #Gyro correction
        if yaw.get_yaw_angle()>desiredDegree+deadband: #original deadband was 5
            if wheelLockRight == True:
                motors.start_tank(0,turnspeed+int(p)+int(i)+int(d))
            elif wheelLockLeft==True:
                motors.start_tank(0-turnspeed-int(p)-int(i)-int(d),0)
            elif dime==True:
                motors.start_tank(0-turnspeed-int(p)-int(i)-int(d),turnspeed+int(p)+int(i)+int(d))
            else:
                motors.start_tank(turnspeed-turndiff,turnspeed+int(p)+int(i)+int(d))
        elif yaw.get_yaw_angle()<desiredDegree-deadband:
            if wheelLockRight == True:
                motors.start_tank(0,0-turnspeed-int(p)-int(i)-int(d))
            elif wheelLockLeft==True:
                motors.start_tank(turnspeed+int(p)+int(i)+int(d),0)
            elif dime == True:
                motors.start_tank(turnspeed+int(p)+int(i)+int(d),0-turnspeed-int(p)-int(i)-int(d))
            else:
                motors.start_tank(turnspeed+int(p)+int(i)+int(d),turnspeed-turndiff)
        else:
            if distance>0:
                motors.start(0,speed=int(a)+rapidness)
            elif distance<0:
                motors.start(0,speed=int(a)-rapidness)
            else:
                motors.stop()
                mb.set_degrees_counted(0)
                mf.set_degrees_counted(0)
                break
                return 0
#function call w/ args
#ka should be around 2-5 (depends on distance higher for short distances low for long distances(like 20+))
#pid straight should be kp = 1, kd = 0, turnspeed = 40, turndiff = 10, deadband = 5. Turn should be default deadband = 0. Turn pid values should be default. same for turnspeed and turndiff.
gyro(90,0,1,kp=1,kd=3,turnspeed=15,turndiff=15)
