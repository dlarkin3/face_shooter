#!/usr/bin/env python

import dynamixel_mx28 as dm28              
import rospy
from geometry_msgs.msg import Vector3


dyna = dm28.dynamixel_mx28(dxl_id=3)
dyna.set_left_limit(3600)
dyna.set_right_limit(1500)
SET_POINT = 320

def callback(data):
    centroid = data.x
    set_speed = 0
    error = SET_POINT - centroid
    if error > 0:
        # move cw
        set_speed = error
    elif error < 0:
        #move cw
        set_speed = 1024 + abs(error)
    else:
        set_speed = 0
    print("SetSpeed: %d, Error: %d" % (set_speed,error))
    dyna.set_moving_speed(int(set_speed))
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('face_shooter', anonymous=True)

    rospy.Subscriber("/face_centroid", Vector3, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

 
