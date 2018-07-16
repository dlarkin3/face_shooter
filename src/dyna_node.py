#!/usr/bin/env python

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
    set_speed = 0
    error = 0
    if data.z ==1: # For now only shoot the first target, ignore others
        centroid = data.x
        set_speed = 0
        error = SET_POINT - centroid
        if error > 0:
            # move cw
            set_speed = error*0.5
        elif error < 0:
            #move ccw
            set_speed = 1024 + abs(error)*0.5

    print("SetSpeed: %d, Error: %d" % (set_speed,error))
    dyna.set_moving_speed(int(set_speed))
    
    
def listener():
    rospy.init_node('face_shooter', anonymous=True)
    rospy.Subscriber("/face_centroid", Vector3, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

 
