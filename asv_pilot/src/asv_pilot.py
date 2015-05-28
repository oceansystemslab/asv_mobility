#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
np.set_printoptions(precision=1, suppress=True)
import sys


from transformations import euler_from_quaternion
import controllers as ctrl

# Messages
from vehicle_interface.msg import PilotRequest, ThrusterCommand
from nav_msgs.msg import Odometry
from vehicle_interface.srv import BooleanService

# Constants
TOPIC_THROTTLE = '/motors/throttle'
TOPIC_ODOMETRY = '/nav/odometry'
TOPIC_WAYPOINT = '/pilot/waypoint'
SRV_SWITCH = '/pilot/switch'
ODOMETRY_TIMEOUT = 5 # seconds
LOOP_RATE = 10  # Hz

class Pilot(object):
    def __init__(self, name, topic_throttle):
        self.name = name

        # latest throttle received
        self.pose = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.des_pose = np.zeros(6)

        self.last_odometry_t = 0
        self.odometry_switch = False
        self.pilot_enable = True

        # Subscribers
        self.waypoint_sub = rospy.Subscriber(TOPIC_WAYPOINT, PilotRequest, self.handle_waypoint)
        self.odometry_sub = rospy.Subscriber(TOPIC_ODOMETRY, Odometry, self.handle_odometry)

        # Publishers
        self.throttle_pub = rospy.Publisher(topic_throttle, ThrusterCommand)

        # Services
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_odometry_t) > ODOMETRY_TIMEOUT and self.odometry_switch:
            self.odometry_switch = False
            rospy.logerr('Odometry outdated')

        if self.odometry_switch and self.pilot_enable:
            throttle = ctrl.point_shoot(self.pose, self.des_pose)
            rospy.loginfo('pose: %s des pose: %s throttles: %s', self.pose, self.des_pose, throttle )
            throttle_msg = ThrusterCommand()
            throttle_msg.header.stamp = rospy.Time().now()
            throttle_msg.throttle = throttle
            self.throttle_pub.publish(throttle_msg)

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

    def handle_switch(self, srv):
        self.pilot_enable = srv.request


if __name__ == '__main__':
    rospy.init_node('asv_pilot')
    name = rospy.get_name()

    topic_throttle = rospy.get_param('~topic_throttle', TOPIC_THROTTLE)

    pilot = Pilot(name, topic_throttle)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            pilot.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        # except Exception as e:
        #     rospy.logfatal('%s', e)
        #     rospy.logfatal('Caught exception and dying!')
        #     sys.exit(-1)


