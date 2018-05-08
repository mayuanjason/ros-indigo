#!/usr/bin/env python

import rospy
from topics.msg import Complex

rospy.init_node('message_subscriber')


def callback(msg):
    print 'Real:', msg.real
    print 'Imaginary:', msg.imaginary
    print


sub = rospy.Subscriber('Complex', Complex, callback)

rospy.spin()
