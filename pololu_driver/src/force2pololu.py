#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('pololu_driver')

import rospy
import sys
import numpy as np

import force_allocator as fa

# Messages
from asv_msgs.msg import ThrusterCommand, FloatArrayStamped

# Services

# Constants
TOPIC_THROTTLE = '/motors/throttle'
TOPIC_FORCE = '/motors/force'
LOOP_RATE = 1  # Hz
MSG_TIMEOUT = 0.5 # seconds


class MotorMapper(object):
    def __init__(self, name):
        self.name = name
        self.i = 0
        self.forces = np.zeros(6)

        # Publishers
        self.throttle_pub = rospy.Publisher(TOPIC_THROTTLE, ThrusterCommand)

        # Subscribers
        self.force_sub = rospy.Subscriber(TOPIC_FORCE, FloatArrayStamped, self.handle_force)

        # Services

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_msg_t) > MSG_TIMEOUT and any(self.throttle):
            self.force = np.zeros(6)
            rospy.loginfo('Force message outdated')

        # Set vertical force, pitch and roll to 0
        self.force[2:4] = 0
        throttle = fa.allocate_force(self.force)
        self.i %= 100

        msg = ThrusterCommand()
        msg.header.stamp = rospy.Time().now()
        msg.throttle = [self.i * 0, 0, 0, 0, 0, 0]

        self.throttle_pub.publish(msg)

    def handle_force(self, msg):
        try:
            self.last_msg_t = msg.header.stamp.to_sec()
            self.force = np.array(msg.values)
        except Exception:
            self.force = np.zeros(6)
            rospy.logerr('Wrong format of the force message')



if __name__ == '__main__':
    rospy.init_node('motor_mapper')
    name = rospy.get_name()
    rospy.loginfo('%s: initializing ...', name)


    node = MotorMapper(name)
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


