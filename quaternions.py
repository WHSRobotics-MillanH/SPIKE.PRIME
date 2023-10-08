from hub import motion_sensor, port
import math, time, motor
def quaterniontoyaww(qi):
    # Extract quaternion components
    w, x, y, z = qi

    # Convert quaternion to Euler angles
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)

    # Convert from radians to degrees
    yaw_degrees = math.degrees(yaw)

    return yaw_degrees
def reset_yaw():
    q = motion_sensor.quaternion()
    return quaterniontoyaww(q)

def yaw(qq, rq):
    q = qq
    return quaterniontoyaww(q) - rq  


motion_sensor.reset_yaw(0)
ryaw = reset_yaw()
while True:
    # Read quaternion from motion sensor
    quaternion = motion_sensor.quaternion()

    # Calculate yaw angle
    robotYaw = -1*yaw(quaternion,ryaw)

    # Display yaw angle
    print("Yaw angle (degrees):", robotYaw)
    time.sleep_ms(1000)
