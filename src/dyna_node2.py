#!/usr/bin/env python
from __future__ import print_function
#from pudb import set_trace; set_trace()

import time
import dynamixel_mx28 as dm28              
import rospy
from geometry_msgs.msg import Vector3
from os.path import expanduser
home = expanduser("~")

timestr = time.strftime("%Y%m%d_%H%M%S")
logfile = home+"/faceshooter_cntrl"+timestr+".csv"

dyna = dm28.dynamixel_mx28(dxl_id=5)
dyna.set_left_limit(3600)
dyna.set_right_limit(1500)
dyna.set_left_limit(800)
dyna.set_right_limit(220) 
SET_POINT = 320 # middle pixel
target_point = Vector3()

#PID Control setup
kP = 1.2 # 1, Increase until oscilates
kI = 0.09
kD = 0.12 # 2. set to stop oscilation start with 10% of kP
lastError = 0
integral = 0


outString = "time,actual,error,effort,P,I,D,kP,kI,kD,set_point,lastError\n"
rospy.loginfo(outString)
with open(logfile, 'a') as outFile:
    outFile.write(outString)

# Each time a target centroid is published then center the camera on it.
def callback(data):
    global target_point
    target_point = data

def find_linear_control(actual, set_point):
    speed = 0
    error = set_point - actual
    if error > 0:
        # move cw
        speed = error*0.5
    elif error < 0:
        #move ccw
        speed = 1024 + abs(error)*0.5
    return speed

def find_pid_control(actual, set_point):
    global lastError
    global integral
    effort = 0
    error = set_point - actual
    if abs(error) < 3:
        integral = 0
    elif abs(error) < 50:
        integral += error
    else:
        integral = 0
    P = error * kP
    I = 0
    I = integral * kI
    D = 0 
    D = (error-lastError)*kD
    effort = (P + I + D)

    outString = "%f,%3d,%3d,%4d,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%d,%4d" % (rospy.get_time(),actual, error, effort, P, I, D, kP, kI, kD, set_point, lastError)
    rospy.loginfo(outString)
    with open((logfile), 'a') as outFile:
        outFile.write(outString+"\n")
    lastError = error
    return effort

if __name__ == '__main__':
    rospy.init_node('face_shooter', anonymous=True)
    rospy.Subscriber("/face_centroid", Vector3, callback)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        set_speed = 0
        if target_point.z == 1: # For now only shoot the first target, ignore others
           # set_speed = find_linear_control(target_point.x,SET_POINT)
           set_speed = find_pid_control(target_point.x,SET_POINT)
        dyna.set_moving_speed2(int(set_speed))
        rate.sleep()


 
