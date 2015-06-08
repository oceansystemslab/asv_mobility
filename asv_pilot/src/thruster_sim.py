#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, Ocean Systems Laboratory, Heriot-Watt University, UK.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Heriot-Watt University nor the names of
#     its contributors may be used to endorse or promote products
#     derived from this software without specific prior written
#     permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Original authors:
#   Valerio De Carolis, Marian Andrecki, Corina Barbalata, Gordon Frost

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
# import sys

import emily_physics as ep

# Messages
from vehicle_interface.msg import ThrusterCommand, Vector6Stamped

# Services
from vehicle_interface.srv import BooleanService

# Constants
TOPIC_THROTTLE = '/motors/throttle'
TOPIC_FORCE = '/body/force'
SRV_SWITCH = '/motors/switch'
MSG_TIMEOUT = 0.5  # seconds
LOOP_RATE = 10  # Hz

class ThrusterSim(object):
    def __init__(self, name, topic_throttle, topic_force):
        self.name = name

        # latest throttle received
        self.throttle = np.zeros(6)
        self.last_msg_t = 0
        self.motor_enable = True

        # Subscribers
        self.throttle_sub = rospy.Subscriber(topic_throttle, ThrusterCommand, self.handle_throttle)

        # Publishers
        self.force_pub = rospy.Publisher(topic_force, Vector6Stamped)

        # Services
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_msg_t) > MSG_TIMEOUT and any(self.throttle):
            self.throttle = np.zeros(6)
            rospy.loginfo('Thruster command outdated')

        if self.motor_enable is True:
            force = ep.compute_body_force(self.throttle)
            msg = Vector6Stamped()
            msg.values = force
            self.force_pub.publish(msg)

    def handle_throttle(self, msg):
        try:
            self.last_msg_t = msg.header.stamp.to_sec()
            self.throttle = np.clip(np.array(msg.throttle[0:6]), -100, 100)
        except Exception:
            rospy.logerr('%s bad input command, skipping!')

    def handle_switch(self, srv):
        self.motor_enable = srv.request
        if not self.motor_enable:
            self.set_force_neutral()

    def set_force_neutral(self):
        msg = Vector6Stamped()
        msg.values = np.zeros(6)
        self.force_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('thruster_sim')
    name = rospy.get_name()

    topic_throttle = rospy.get_param('~topic_throttle', TOPIC_THROTTLE)
    topic_force = rospy.get_param('~topic_force', TOPIC_FORCE)

    rospy.loginfo('throttle topic: %s', topic_throttle)
    rospy.loginfo('force topic: %s', topic_force)

    ts = ThrusterSim(name, topic_throttle, topic_force)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            ts.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            ts.set_force_neutral()
            rospy.loginfo('%s caught ros interrupt!', name)
        # except Exception as e:
        #     ts.set_force_neutral()
        #     rospy.logfatal('%s', e)
        #     rospy.logfatal('%s caught exception and dying!', name)
        #     sys.exit(-1)



