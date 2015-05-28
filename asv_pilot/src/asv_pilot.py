#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
np.set_printoptions(precision=3, suppress=True)
import sys


from transformations import euler_from_quaternion
import controllers as ctrl

# Messages
from vehicle_interface.msg import PilotRequest, ThrusterCommand
from nav_msgs.msg import Odometry

# Services

# Constants
TOPIC_WAYPOINT = '/path/waypoint'
TOPIC_THROTTLE = '/motors/throttle'
ODOMETRY_NAV = '/nav/odometry'
ODOMETRY_TIMEOUT = 5 # seconds
LOOP_RATE = 10  # Hz

# TODO: add service for disabling/enabling the pilot
# TODO: convert constants to rosparams
class Pilot(object):
    def __init__(self, name):
        self.name = name

        # latest throttle received
        self.pose = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.des_pose = np.zeros(6)

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
        if (rospy.Time.now().to_sec() - self.last_odometry_t) > ODOMETRY_TIMEOUT and self.odometry_switch:
            self.odometry_switch = False
            rospy.logerr('Odometry outdated')

        rospy.loginfo('pose: %s des pose: %s', self.pose, self.des_pose )

        if self.odometry_switch is True:
            throttle = ctrl.point_shoot(self.pose, self.des_pose)
            rospy.logdebug('throttles: %s', throttle)
            throttle_msg = ThrusterCommand()
            throttle_msg.header.stamp = rospy.Time().now()
            throttle_msg.throttle = throttle

    def handle_odometry(self, msg):
        try:
            pos = msg.pose.pose.position
            self.pose[0:3] = np.array([pos.x, pos.y, pos.z])
            rot = msg.pose.pose.orientation
            quaternion = np.array([rot.w, rot.x, rot.y, rot.z])
            self.pose[3:6] = euler_from_quaternion(quaternion)
            self.last_odometry_t = msg.header.stamp.to_sec()
            self.odometry_switch = True
        except Exception as e:
            rospy.logerr('%s', e)
            rospy.logerr('Bad odometry message format, skipping!')

    def handle_waypoint(self, msg):
        try:
            self.des_pose = np.array(msg.position)
            # ignore depth, pitch and roll
            self.des_pose[2:5] = 0
        except Exception as e:
            rospy.logerr('%s', e)
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
            rospy.logfatal('%s', e)
            rospy.logfatal('Caught exception and dying!', e)
            sys.exit(-1)


