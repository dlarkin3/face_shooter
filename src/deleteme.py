#!/usr/bin/env python
from __future__ import print_function
#from pudb import set_trace; set_trace()

import time
import dynamixel_mx28 as dm28              
import rospy
from geometry_msgs.msg import Vector3
from os.path import expanduser
from multiprocessing import Process, Queue

home = expanduser("~")
timestr = time.strftime("%Y%m%d_%H%M%S")
outString = "time,actual,error,effort,P,I,D,kP,kI,kD,set_point,lastError\n"
rospy.loginfo(outString)

def control_loop(desired_que):
    tlast_time = time.time()
    tnum_frames = 0
    rate2 = rospy.Rate(500) # 10hz
    terror = 0
    while not rospy.is_shutdown():
        try:
            terror = desired_que.get_nowait()
        except:
            #rospy.loginfo("ERROR: Control Process found an empty queue")
            pass
        tcurr_time = time.time()
        tnum_frames += 1
        telapsed_time = tcurr_time - tlast_time        
        if (telapsed_time > 1.0):
            rospy.loginfo("PROC FPS: %f math is %d / %f == %.2f = %d" % ((tnum_frames / telapsed_time),tnum_frames,telapsed_time,terror,desired_que.qsize()))
            tlast_time = tcurr_time
            tnum_frames = 0
        rate2.sleep()

if __name__ == '__main__':
    rospy.init_node('face_shooter', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    last_time = time.time()
    num_frames = 0
    target_queue = Queue()
    p1 = Process(target=control_loop, args=(target_queue,))
    p1.start()
    error = 0.0
    while not rospy.is_shutdown():
        curr_time = time.time()
        num_frames += 1
        error += 0.1
        target_queue.put(error)
        elapsed_time = curr_time - last_time
        if (elapsed_time > 1.0):
            rospy.loginfo("MAIN FPS: %f math is %d / %f" % ((num_frames / elapsed_time),num_frames,elapsed_time))
            last_time = curr_time
            num_frames = 0
        rate.sleep()
    target_queue.close()
    target_queue.join_thread()    
    p1.join()


 
