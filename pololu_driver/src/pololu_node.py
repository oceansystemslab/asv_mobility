#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('pololu_driver')

import rospy
import sys
import numpy as np

import pololu_protocol

# Messages
from asv_msgs.msg import ThrusterCommand

# Services

# Constants
PORT = '/dev/ttyACM0'
TOPIC_THROTTLE = '/motors/throttle'
MSG_TIMEOUT = 0.5 # seconds
LOOP_RATE = 10  # Hz


class PololuNode(object):
    def __init__(self, name, port):
        self.name = name
        self.pololu = pololu_protocol.PololuIF(port)

        # latest throttle received
        self.throttle = np.zeros(6)
        self.last_msg_t = 0
        self.motor_switch = True

        # Subscribers
        self.throttle_sub = rospy.Subscriber(TOPIC_THROTTLE, ThrusterCommand, self.handle_throttle)

        # Services

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_msg_t) > MSG_TIMEOUT and any(self.throttle):
            self.throttle = np.zeros(6)
            rospy.loginfo('Thruster command outdated')

        if self.motor_switch is True:
            for servo in range(0, 2):
                if self.pololu.set_servo(servo, self.throttle[servo]) > 0:
                    rospy.logerr('Error writing to Pololu')

    def handle_throttle(self, msg):
        # self.last_msg_t = rospy.Time.now().to_sec()
        try:
            self.last_msg_t = msg.header.stamp.to_sec()
            self.throttle = np.clip(np.array(msg.throttle[0:6]), -100, 100)
        except Exception:
            rospy.logerr('%s bad input command, skipping!')


if __name__ == '__main__':
    rospy.init_node('pololu_driver')
    name = rospy.get_name()

    node = PololuNode(name,PORT)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            node.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
            node.pololu.set_all_neutral()
            node.pololu.port.close()
        except Exception as e:
            rospy.logfatal('%s caught exception and dying!', name)
            node.pololu.set_all_neutral()
            node.pololu.port.close()
            sys.exit(-1)

