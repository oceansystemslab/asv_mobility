#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
from transformations import euler_from_quaternion

import controllers as ctrl

# Messages
from asv_msgs.msg import PilotRequest, ThrusterCommand
from nav_msgs.msg import Odometry

# Services

# Constants
TOPIC_WAYPOINT = '/path/waypoint'
TOPIC_THROTTLE = '/motors/throttle'
ODOMETRY_NAV = '/nav/odometry'
ODOMETRY_TIMEOUT = 0.5 # seconds
LOOP_RATE = 10  # Hz


class Pilot(object):
    def __init__(self, name):
        self.name = name

        # latest throttle received
        self.pos = np.zeros(6)
        self.des_pos = np.zeros(6)

        self.last_odometry_t = 0
        self.odometry_switch = False

        # Subscribers
        self.waypoint_sub = rospy.Subscriber(TOPIC_WAYPOINT, PilotRequest, self.handle_waypoint)
        self.odometry_sub = rospy.Subscriber(ODOMETRY_NAV, Odometry, self.handle_odometry)

        # Publishers
        self.throttle_pub = rospy.Publisher(TOPIC_THROTTLE, ThrusterCommand)

        # Services

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_odometry_t) > ODOMETRY_TIMEOUT:
            self.odometry_switch = False
            rospy.logerr('Odometry outdated')

        if self.odometry_switch is True:
            throttle = ctrl.compute_throttle(self.pos, self.des_pos)
            throttle_msg = ThrusterCommand()
            throttle_msg.header.stamp = rospy.Time().now()
            throttle_msg.throttle = throttle

    def handle_odometry(self, msg):
        try:
            print 'position', msg.pose.pose.position
            self.pos[0:3] = np.array(msg.pose.pose.position)
            rotation = euler_from_quaternion(np.array(msg.pose.pose.orientation))
            print 'orientation', rotation
            self.pos[3:6] = rotation
            self.last_odometry_t = msg.header.stamp.to_sec()
            self.odometry_switch = True
        except Exception as e:
            rospy.logerr('%s Bad odometry message format, skipping!', e)

    def handle_waypoint(self, msg):
        try:
            self.des_pos = np.clip(np.array(msg.throttle[0:6]), -100, 100)
        except Exception:
            rospy.logerr('Bad waypoint message format, skipping!')


if __name__ == '__main__':
    rospy.init_node('asv_pilot')
    name = rospy.get_name()

    node = Pilot(name)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            node.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        except Exception as e:
            rospy.logfatal('%s caught exception and dying!', name)


