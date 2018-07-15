#!/usr/bin/env python

import rospy
import cv2
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from numpy import interp

# Update target location from ros subscription
x_correction = Float64() # start location is 0.0

def target_callback(msg):
    global target_loc
    target_loc = float(msg.x)
    pub_position_corrections1(target_loc, img_center)

def pub_position_corrections0(target, center):
    global x_correction
    error = get_position_error(target, center)
    if (error.x < 0):
        x_correction.data += step_distancex
        #output_str = "Less then center correction: %f, " % (x_correction.data)
        #rospy.loginfo(output_str)
    else:
        x_correction.data -= step_distancex
        #output_str = "Greater then center correction: %f" % (x_correction.data)
        #rospy.loginfo(output_str)
    pub_pan.publish(x_correction)

def pub_position_corrections1(target, center):
    global x_correction
    this_correction = interp(target,[0,640],[-0.151,0.151])
    x_correction.data += this_correction
    pub_pan.publish(x_correction)
    output_str = "TargerPix, Correction, controlPos: %.1f, %.2f, %.2f" % (target,this_correction,x_correction.data)
    rospy.loginfo(output_str)

def get_position_error(actual, set_point):
    error = Vector3()
    error.x = set_point.x - actual.x 
    error.y = set_point.y - actual.y
    return error
    
if __name__ == '__main__':
    pub_pan = rospy.Publisher('/pan_controller/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/tilt_controller/command', Float64, queue_size=1)
    rospy.init_node("face_shooter") 

    # Load params if provided else use the defaults
    center_offset = rospy.get_param("center_offset","100")
    screenmaxx = rospy.get_param("screenmaxx","640")
    screenmaxy = rospy.get_param("screenmaxy","480")
    servomaxx = rospy.get_param("servomaxx","0.5")
    servomaxy = rospy.get_param("servomaxy","0.5")
    servomin = rospy.get_param("servomin","-0.5")
    step_distancex = float(rospy.get_param("step_distancex","0.005"))
    step_distancey = float(rospy.get_param("step_distancey","0.005"))
    img_center = Vector3()
    img_center.x = screenmaxx/2
    img_center.y = screenmaxy/2
    target_sub = rospy.Subscriber("/face_centroid", Vector3, target_callback) 
    pub_pan.publish(x_correction)

    rate = rospy.Rate(5) # 20hz
    while not rospy.is_shutdown():
       

        rate.sleep()
