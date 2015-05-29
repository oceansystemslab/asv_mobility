#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
import sys

import emily_physics as ep

# Messages
from vehicle_interface.msg import ThrusterCommand, FloatArrayStamped

# Services
from vehicle_interface.srv import BooleanService

# Constants
TOPIC_THROTTLE = '/motors/throttle'
TOPIC_FORCE = '/body/force'
SRV_SWITCH = '/motors/switch'
MSG_TIMEOUT = 0.5  # seconds
LOOP_RATE = 10  # Hz

# TODO: convert constants to rosparams
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
        self.force_pub = rospy.Publisher(topic_force, FloatArrayStamped, tcp_nodelay=True, queue_size=1)

        # Services
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_msg_t) > MSG_TIMEOUT and any(self.throttle):
            self.throttle = np.zeros(6)
            rospy.loginfo('Thruster command outdated')

        if self.motor_enable is True:
            force = ep.compute_body_force(self.throttle)
            msg = FloatArrayStamped()
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
        msg = FloatArrayStamped()
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
        except Exception as e:
            ts.set_force_neutral()
            rospy.logfatal('%s', e)
            rospy.logfatal('%s caught exception and dying!', name)
            sys.exit(-1)



