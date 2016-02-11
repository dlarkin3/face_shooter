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
logfile = home+"/faceshooter_pid"+timestr+".csv"


dyna = dm28.dynamixel_mx28(dxl_id=5)
dyna.set_left_limit(3600)
dyna.set_right_limit(1500)
dyna.set_left_limit(800)
dyna.set_right_limit(220) 
SET_POINT = 320 # middle pixel
target_point = Vector3()

#PID Control setup
kP = 0.95 # 1, Increase until oscilates
kI = 0.0138
kD = 0.5 # 2. set to stop oscilation
lastError = 0


outString = "actual,set_point,error,lastError,speed,P,I,D,kP,kI,kD\n"
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
    speed = 0
    error = set_point - actual
    P = abs(error) * kP
    I = 0
    D = 0 
    #D = (error-lastError)*kD
    output = P + I + D

    # Account for direction CW or CCW
    if error > 0:        # move cw
        speed = output
    elif error < 0:      #move ccw
        speed = 1024 + output
    outString = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d" % (actual, set_point, error, lastError, speed, P, I, D, kP, kI, kD)
    rospy.loginfo(outString)
    with open((logfile), 'a') as outFile:
        outFile.write(outString+"\n")
    lastError = error
    return speed

if __name__ == '__main__':
    rospy.init_node('face_shooter', anonymous=True)
    rospy.Subscriber("/face_centroid", Vector3, callback)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        set_speed = 0
        if target_point.z == 1: # For now only shoot the first target, ignore others
           # set_speed = find_linear_control(target_point.x,SET_POINT)
           set_speed = find_pid_control(target_point.x,SET_POINT)
        dyna.set_moving_speed(int(set_speed))
        rate.sleep()


 
