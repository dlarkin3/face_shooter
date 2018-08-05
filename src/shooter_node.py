#!/usr/bin/env python
from __future__ import print_function
#from pudb import set_trace; set_trace()

import time
import dynamixel_mx28 as dm28              
import rospy
from geometry_msgs.msg import Vector3
from os.path import expanduser
from multiprocessing import Process, Manager

home = expanduser("~")

timestr = time.strftime("%Y%m%d_%H%M%S")
logfile = home+"/faceshooter_cntrl"+timestr+".csv"

dyna = dm28.dynamixel_mx28(dxl_id=5)
dyna.set_left_limit(3600)
dyna.set_right_limit(1500)
dyna.set_left_limit(800)
dyna.set_right_limit(220) 
SET_POINT = 320 # middle pixel

target_error = 0 # global for communication with callback

#PID Control setup
kP = 1.0 # 1, Increase until oscilates
kI = 0.09
kD = 0.025 # 2. set to stop oscilation start with 10% of kP
integral = 0.0

outString = "time,error,effort,P,I,D,kP,kI,kD,laserror\n"
rospy.loginfo(outString)
with open(logfile, 'a') as outFile:
    outFile.write(outString)

# Each time a target centroid is published then add error to the que 
def callback(data):
    global target_error
    if (data.z == 1):
        target_error = SET_POINT - data.x
        #print("CB: target error; %f" %target_error)
    else: # there is no face
        target_error = 99999 # use as a flag to not shoot and not move 

def find_linear_control(error, last_error):
    speed = 0
    if error > 0:
        # move cw
        speed = error*0.7
    elif error < 0:
        #move ccw
        speed = 1024 + abs(error)*0.7
    return speed

# Given error and last error find the control effort
def find_pid_control(error, last_error):
    global integral
    if abs(error) < 3:
        integral = 0
    elif abs(error) < 50:
        integral += error
    else:
        integral = 0
    P = error * kP
    I = 0
    #I = integral * kI
    D = 0 
    #D = (error-last_error)*kD
    effort = (P + I + D)
    outString = "%f,%3d,%4d,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%4d" % (rospy.get_time(),error, effort, P, I, D, kP, kI, kD, last_error)
    #rospy.loginfo(outString)
    with open((logfile), 'a') as outFile:
        outFile.write(outString+"\n")
    return effort  
    
def shoot_gun(trigger):
    start_time = time.time()
    last_flag = 0
    rate2 = rospy.Rate(30) # 10hz
    state = 0
    while not rospy.is_shutdown():
        trig_flag = trigger[0]
        with open('/sys/class/gpio/gpio388/value', 'w') as f:
            f.write(str(trig_flag))               
        rate2.sleep()        

if __name__ == '__main__':
    rospy.init_node('face_shooter', anonymous=True)
    rospy.Subscriber("/face_centroid", Vector3, callback)

    last_error = 0
    target_error = 99999 # This is the gloabl target_error updated by the callback this value is a do nothing flag
    integral = 0
    control_effort = 0
    last_time = time.time()
    num_frames = 0
    rate = rospy.Rate(100) # The max speed while setting speed on 1 dynamixel motor was 250 hz.

    mgr = Manager()
    trigger = mgr.list([0,])
    
    p1 = Process(target=shoot_gun, args=(trigger,))
    p1.start()
    
    #dyna.set_moving_speed(300,1)      
    while not rospy.is_shutdown():
        if target_error != 99999:  
            #control_effort = find_pid_control(target_error,last_error)
            control_effort = find_linear_control(target_error,last_error)
            dyna.set_moving_speed2(int(control_effort))      
            # This section is used to watch timing for development purposes.
            if (-15 < target_error < 15):
                trigger[0] = 1
            else:
                trigger[0] = 0
        else:
            dyna.set_moving_speed2(0)
            trigger[0] = 0
        curr_time = time.time()
        num_frames += 1
        elapsed_time = curr_time - last_time
        if (elapsed_time > 5.0):
            print("%3.2f, %3.2f == FPS: %f" % (target_error,control_effort, (num_frames / elapsed_time)))
            last_time = curr_time
            num_frames = 0
        last_error = target_error

        rate.sleep()
    p1.join()

'''
rostopic pub -r 10 /face_centroid geometry_msgs/Vector3  '{x: 220, y: 0.0, z: 1.0}'
rostopic pub -r 10 /face_centroid geometry_msgs/Vector3  '{x: 320, y: 0.0, z: 1.0}'
'''
 
