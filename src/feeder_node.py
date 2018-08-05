#!/usr/bin/env python
from __future__ import print_function

import time
import dynamixel_mx28 as dm28              
import rospy
from geometry_msgs.msg import Vector3
from os.path import expanduser
from multiprocessing import Process, Manager
SET_POINT = 320 # middle pixel

dyna = dm28.dynamixel_mx28(dxl_id=1) 
target_error = 0 # global for communication with callback

# Each time a target centroid is published then add error to the que 
def callback(data):
    global target_error
    if (data.z == 1):
        target_error = SET_POINT - data.x
        #print("CB: target error; %f" %target_error)
    else: # there is no face
        target_error = 99999 # use as a flag to not shoot and not move 
        
if __name__ == '__main__':
    rospy.init_node('feeder_node', anonymous=True)
    rate = rospy.Rate(20) # The max speed while setting speed on 1 dynamixel motor was 250 hz.
    rospy.Subscriber("/face_centroid", Vector3, callback)
    dyna.set_moving_speed3(300,1) #start loading
    time.sleep(1)
    while not rospy.is_shutdown():
        set_speed = 0
        current_speed=dyna.get_present_speed()
        if (current_speed < 10): # if fully loaded then stop
            dyna.set_torque()
        if target_error != 99999: # if shooting start loading again
            if (-15 < target_error < 15):
                set_speed = 300
        dyna.set_moving_speed3(set_speed,1)

        rate.sleep()

 
