#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import roslib
roslib.load_manifest('pololu_driver')

import sys
import numpy as np

import pololu_protocol

# Messages
from asv_msgs.msg import ThrusterCommand

# Services

# Constants
PORT = '/dev/ttyACM0'
TOPIC_THROTTLE = '/motors/throttle'
WATCHDOG_PERIOD = 0.5
LOOP_RATE = 10  # Hz


class PololuNode(object):
    def __init__(self, name, port):
        self.name = name
        self.pololu = pololu_protocol.PololuIF(port)

        # latest throttle received
        self.throttle = np.zeros(6)

        # Subscribers
        self.throttle_sub = rospy.Subscriber(TOPIC_THROTTLE, ThrusterCommand, self.handle_throttle)

        # Services

        # Timers
        self.watchdog = rospy.Timer(rospy.Duration(WATCHDOG_PERIOD), self.handle_watchdog, oneshot=True)

    def loop(self):
        for servo in range(0, 2):
            self.pololu.set_servo(servo, self.throttle[servo])

    def handle_throttle(self, msg):
        try:
            self.throttle = np.clip(np.array(msg.throttle[0:6]), -100, 100)
            self.clear_watchdog()
        except Exception:
            rospy.logerr('%s bad input command, skipping!')

    def handle_watchdog(self):
        self.throttle = np.zeros(6)
        self.pololu.set_all_neutral()

    def clear_watchdog(self):
        self.watchdog.shutdown()
        self.watchdog = rospy.Timer(rospy.Duration(WATCHDOG_PERIOD), self.handle_watchdog, oneshot=True)


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
        except Exception as e:
            rospy.logfatal('%s caught exception and dying!', name)
            node.pololu.set_all_neutral()
            sys.exit(-1)


