#!/usr/bin/env python
import rospy
import std_msgs
import time

def talker():
    pub = rospy.Publisher('Direction', std_msgs.msg.String, queue_size=10)
    pub2 = rospy.Publisher('Speed', std_msgs.msg.Float32, queue_size=10)
    rospy.init_node('arduino_sender')

    #pub.publish('forward')
    time.sleep(1)
    pub2.publish(0)


talker()
