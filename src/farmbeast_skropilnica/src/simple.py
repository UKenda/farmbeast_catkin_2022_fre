#! /usr/bin/python3

import rospy

import os
from std_msgs.msg import Empty


if __name__=="__main__":
    
    rospy.init_node('skropilnica_node')
    rospy.set_param("/farmbeast/skropilnica/S1S2", False)
    rospy.set_param("/farmbeast/skropilnica/S3S4", False)

    #odrv0.axis1.requested_state = 8 #AXIS_STATE_CLOSED_LOOP_CONTROLL
    pubS1S2 = rospy.Publisher('S1S2', Empty, queue_size=1)
    pubS3S4 = rospy.Publisher('S3S4', Empty, queue_size=1)
    
    

    while(True):
        if(rospy.get_param("/farmbeast/skropilnica/S1S2")):
            rospy.set_param("/farmbeast/skropilnica/S1S2", False)
            pubS1S2.publish()
    rospy.on_shutdown(turn_off)
    