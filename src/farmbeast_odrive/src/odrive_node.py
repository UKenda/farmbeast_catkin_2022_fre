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
flag = True
flag_tekmovanje = False
def joy_callback(data):
    global flag, rotate
    #print(data)
    if data.buttons[7]:
        print("Calibration")
        dump_errors(odrv0,True)
        dump_errors(odrv1,True)
        if(rospy.get_param("/farmbeast/wheel/calibration/all")):
            print("all")
            odrv0.axis0.requested_state = 3 #FULL_CALIBRATION_SEQUENCE
            odrv0.axis1.requested_state = 3 
            odrv1.axis0.requested_state = 3 
            odrv1.axis1.requested_state = 3
            rospy.set_param("/farmbeast/wheel/calibration/all",False)
        elif (rospy.get_param("/farmbeast/wheel/calibration/the_one")):
            print("the one")
            odrv1.axis0.requested_state = 3 
            rospy.set_param("/farmbeast/wheel/calibration/the_one",False)
        else:
            print("Not")

    if data.buttons[5] and flag == False:
        print("AXIS_STATE_IDLE")
        odrv0.axis0.requested_state = 1 #AXIS_STATE_IDLE
        odrv0.axis1.requested_state = 1 
        odrv1.axis0.requested_state = 1 
        odrv1.axis1.requested_state = 1
      
        flag=True
    elif data.buttons[5] and flag == True:
        print("AXIS_STATE_CLOSED_LOOP_CONTROLL")
        odrv0.axis0.requested_state = 8 #AXIS_STATE_CLOSED_LOOP_CONTROLL
        odrv0.axis1.requested_state = 8 
        odrv1.axis0.requested_state = 8 
        odrv1.axis1.requested_state = 8

        flag = False
    
    if data.axes[0] == 0:
        rotate = False
    if rospy.get_param("/farmbeast/drive_mode") == 1: # 0 = calibration_mode   1 = joy_mode | 2 = navigation_mode 
        if data.axes[0] > 0:
            dump_errors(odrv0,True)
            dump_errors(odrv1,True)
            rotate = True
            odrv0.axis0.controller.input_vel = 0.5
            odrv0.axis1.controller.input_vel = 0.5
            odrv1.axis0.controller.input_vel = 0.5
            odrv1.axis1.controller.input_vel = 0.5
        elif data.axes[0] < 0:
            dump_errors(odrv0,True)
            dump_errors(odrv1,True)
            rotate = True
            odrv0.axis0.controller.input_vel = -0.5
            odrv0.axis1.controller.input_vel = -0.5
            odrv1.axis0.controller.input_vel = -0.5
            odrv1.axis1.controller.input_vel = -0.5
    

def cmd_callback(data):
    global rotate
    global flag_tekmovanje
    obracanje = rospy.get_param("/farmbeast/obracanje")

    if rospy.get_param("/farmbeast/idle/tekmovanje"):
        rospy.set_param("/farmbeast/idle/tekmovanje", False)
        odrv0.axis0.requested_state = 1 #AXIS_STATE_IDLE
        odrv0.axis1.requested_state = 1 
        odrv1.axis0.requested_state = 1 
        odrv1.axis1.requested_state = 1
        flag_tekmovanje = True
    elif flag_tekmovanje and not flag:
        flag_tekmovanje = False
        print("idle")
        odrv0.axis0.requested_state = 8 #AXIS_STATE_IDLE
        odrv0.axis1.requested_state = 8 
        odrv1.axis0.requested_state = 8 
        odrv1.axis1.requested_state = 8

    if obracanje == 1:
        print("levo")
        odrv0.axis0.controller.input_vel = -0.75
        odrv0.axis1.controller.input_vel = -0.75
        odrv1.axis0.controller.input_vel = -0.75
        odrv1.axis1.controller.input_vel = -0.75
    elif obracanje == 2:
        print("desno")
        odrv0.axis0.controller.input_vel = 0.75
        odrv0.axis1.controller.input_vel = 0.75
        odrv1.axis0.controller.input_vel = 0.75
        odrv1.axis1.controller.input_vel = 0.75
    else :   
        print("cmdvel")
        speed = rospy.get_param("/farmbeast/wheel/speed")*2
        if not rotate:

            odrv0.axis0.controller.input_vel = -(speed*data.linear.x - data.angular.z*data.linear.x*1.5)
            odrv0.axis1.controller.input_vel = -(speed*data.linear.x - data.angular.z*data.linear.x*1.5)
            odrv1.axis0.controller.input_vel = speed*data.linear.x + data.angular.z*data.linear.x*1.5
            odrv1.axis1.controller.input_vel = speed*data.linear.x + data.angular.z*data.linear.x*1.5
            
def turn_off():
    odrv0.axis0.requested_state = 1 #AXIS_STATE_IDLE
    odrv0.axis1.requested_state = 1 
    odrv1.axis0.requested_state = 1 
    odrv1.axis1.requested_state = 1

    print("bye")



if __name__=="__main__":
    
    rospy.init_node('odrive_node')

    odrv0.axis0.requested_state = 1 #AXIS_STATE_IDLE
    odrv0.axis1.requested_state = 1 
    odrv1.axis0.requested_state = 1 
    odrv1.axis1.requested_state = 1
    rospy.set_param("/farmbeast/idle/tekmovanje", False)
    #odrv0.axis1.requested_state = 8 #AXIS_STATE_CLOSED_LOOP_CONTROLL
    '''
    i=0
    while i <5:
        odrv0.axis1.controller.input_vel = 1

        time.sleep(2)
        odrv0.axis1.controller.input_vel = 0
        time.sleep(2)
        i = i+1'''
    rospy.Subscriber("cmd_vel",Twist,cmd_callback,queue_size=1)
    rospy.Subscriber("joy",Joy, joy_callback,queue_size=1)

    rospy.spin()
    rospy.on_shutdown(turn_off)
    