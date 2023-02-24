#! /usr/bin/python3

import rospy
import odrive
from odrive.utils import * 
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
odrv1 = odrive.find_any(serial_number="3372366B3539")
print(odrv1.serial_number)
odrv0 = odrive.find_any(serial_number="335D36593539")
print(odrv0.serial_number)
speed=0.5

rotate = True
flag = False
flag_tekmovanje = True



if __name__=="__main__":
    
    rospy.init_node('odrive_idle_node')

    rospy.set_param("/farmbeast/idle/tekmovanje", False)
    #odrv0.axis1.requested_state = 8 #AXIS_STATE_CLOSED_LOOP_CONTROLL
    while(True):
        if rospy.get_param("/farmbeast/idle/tekmovanje"):
            rospy.set_param("/farmbeast/idle/tekmovanje", False)
            odrv0.axis0.requested_state = 1 #AXIS_STATE_IDLE
            odrv0.axis1.requested_state = 1 
            odrv1.axis0.requested_state = 1 
            odrv1.axis1.requested_state = 1
            flag = True
        elif flag:
            flag = False
            odrv0.axis0.requested_state = 8 #AXIS_STATE_IDLE
            odrv0.axis1.requested_state = 8 
            odrv1.axis0.requested_state = 8 
            odrv1.axis1.requested_state = 8


    rospy.spin()
    rospy.on_shutdown(turn_off)
    