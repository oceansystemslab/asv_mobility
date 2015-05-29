#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

"""Controls the motors on Emily via Pololu Maestro microcontroller. Consumes throttle commands. Thruster accepts
throttles ranging from 0 to 100. Rudder controlling servo accepts values from -100 to 100. At -100 the rudder is
forcing water jet to the right of the vehicle (i.e. vehicle starts turning right).
"""

import roslib
roslib.load_manifest('pololu_driver')

import rospy
import sys
import numpy as np

import pololu_protocol

# Messages
from vehicle_interface.msg import ThrusterCommand

# Services
from vehicle_interface.srv import BooleanService

# Constants
PORT = '/dev/ttyACM0'
TOPIC_THROTTLE = '/motors/throttle'
SRV_SWITCH = '/motors/switch'
MSG_TIMEOUT = 0.5  # seconds
LOOP_RATE = 20  # Hz

class PololuNode(object):
    def __init__(self, name, port):
        self.name = name
        self.pololu = pololu_protocol.PololuIF(port)

        # latest throttle received
        self.throttle = np.zeros(6)
        self.last_msg_t = 0
        self.motor_enable = True

        # Subscribers
        self.throttle_sub = rospy.Subscriber(TOPIC_THROTTLE, ThrusterCommand, self.handle_throttle)

        # Services
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_msg_t) > MSG_TIMEOUT and any(self.throttle):
            self.throttle = np.zeros(6)
            rospy.loginfo('Thruster command outdated')

        if self.motor_enable is True:
            for servo in range(0, 2):
                if self.pololu.set_servo(servo, self.throttle[servo]) > 0:
                    rospy.logerr('Error writing to Pololu')

    def handle_throttle(self, msg):
        try:
            self.last_msg_t = msg.header.stamp.to_sec()
            self.throttle = np.clip(np.array(msg.throttle[0:6]), -100, 100)
        except Exception:
            rospy.logerr('%s bad input command, skipping!')

    def handle_switch(self, srv):
        self.motor_enable = srv.request
        if not self.motor_enable:
            self.throttle = np.zeros(6)
            self.pololu.set_all_neutral()

if __name__ == '__main__':
    rospy.init_node('pololu_driver')
    name = rospy.get_name()

    port = rospy.get_param('~port', PORT)

    node = PololuNode(name, port)
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
            node.pololu.set_all_neutral()
            node.pololu.port.close()
            rospy.logfatal('%s', e)
            rospy.logfatal('%s caught exception and dying!', name)
            sys.exit(-1)


