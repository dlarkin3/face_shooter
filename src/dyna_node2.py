#!/usr/bin/env python
from __future__ import print_function

import dynamixel_mx28 as dm28              
import rospy
from geometry_msgs.msg import Vector3

dyna = dm28.dynamixel_mx28(dxl_id=1)
dyna.set_left_limit(3600)
dyna.set_right_limit(1500)
dyna.set_left_limit(220)
dyna.set_right_limit(800)
SET_POINT = 320
target_point = Vector3()

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
    #rospy.loginfo("Target: %d, speed: %d, Error: %d" % (target_point.z,speed,error))
    return speed



if __name__ == '__main__':
    rospy.init_node('face_shooter', anonymous=True)
    rospy.Subscriber("/face_centroid", Vector3, callback)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        set_speed = 0
        if target_point.z ==1: # For now only shoot the first target, ignore others
            #rospy.loginfo("HERENOW1")
            set_speed = find_linear_control(target_point.x,SET_POINT)
            #rospy.loginfo("HERENOW2")
        dyna.set_moving_speed(int(set_speed))
        #rospy.loginfo("HERENOW3")
        rate.sleep()
        rospy.loginfo("HERENOW4 %d %d" % (set_speed, target_point.x))

 
