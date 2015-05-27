#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
np.set_printoptions(precision=3, suppress=True)


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
ODOMETRY_TIMEOUT = 5 # seconds
LOOP_RATE = 10  # Hz


class Pilot(object):
    def __init__(self, name):
        self.name = name

        # latest throttle received
        self.pos = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
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
        if (rospy.Time.now().to_sec() - self.last_odometry_t) > ODOMETRY_TIMEOUT and self.odometry_switch:
            self.odometry_switch = False
            rospy.logerr('Odometry outdated')

        rospy.loginfo('POS: %s DES_POS: %s', self.pos, self.des_pos )

        if self.odometry_switch is True:
            throttle = np.zeros(6)
            # throttle = ctrl.compute_throttle(self.pos, self.des_pos)
            throttle_msg = ThrusterCommand()
            throttle_msg.header.stamp = rospy.Time().now()
            throttle_msg.throttle = throttle

    def handle_odometry(self, msg):
        try:
            pos = msg.pose.pose.position
            self.pos[0:3] = np.array([pos.x, pos.y, pos.z])
            rot = msg.pose.pose.orientation
            quaternion = np.array([rot.w, rot.x, rot.y, rot.z])
            self.pos[3:6] = euler_from_quaternion(quaternion)
            self.last_odometry_t = msg.header.stamp.to_sec()
            self.odometry_switch = True
        except Exception as e:
            rospy.logerr('Bad odometry message format, skipping!')

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
            rospy.logfatal('%s caught exception and dying!', e)


