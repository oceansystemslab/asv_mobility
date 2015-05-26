#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('pololu_driver')

import rospy
import sys
import numpy as np

# Messages
from asv_msgs.msg import ThrusterCommand

# Services

# Constants
TOPIC_THROTTLE = '/motors/throttle'
LOOP_RATE = 4  # Hz


class MotorMapper(object):
    def __init__(self, name):
        self.name = name
        self.i = 0

        # Subscribers
        self.throttle_pub = rospy.Publisher(TOPIC_THROTTLE, ThrusterCommand)

        # Services

    def loop(self):
        self.i += 10
        self.i %= 100
        msg = ThrusterCommand()
        msg.throttle = [self.i, 0, 0, 0, 0, 0]
        self.throttle_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('motor_mapper')
    name = rospy.get_name()

    node = MotorMapper(name,)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            node.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        except Exception as e:
            rospy.logfatal('%s caught exception and dying!', name)
            sys.exit(-1)


