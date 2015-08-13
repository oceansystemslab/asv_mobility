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

from __future__ import division

import roslib
roslib.load_manifest('emily_nav')

import rospy
import numpy as np
np.set_printoptions(precision=1, suppress=True)

import frame_maths as frame

# Messages
from auv_msgs.msg import NavSts
from xsens.msg import Xsens
from vehicle_interface.srv import BooleanService, BooleanServiceResponse
# xsens message

# Constants
R_EARTH = 6371000  # metres - average radius of Earth

TOPIC_NAV = '/nav/nav_sts'
TOPIC_LOW_FREQ = '/modem/packer/nav_sts'
TOPIC_XSENS = '/imu/xsens'
SRV_RESET_ORIGIN = '/nav/reset'
LOW_FREQ = 0.1  # Hz
ZERO_PITCH_ROLL = True

SENSOR_ANGLE_OFFSETS = np.array([0, 0, 0])

class Navigation(object):
    def __init__(self, name, topic_nav, topic_low_freq, origin, wait_for_GPS, zero_pitch_roll, **kwargs):
        self.name = name

        self.point_ll = np.zeros(2)  # [latitude, longitude]
        self.displacement_ne = np.zeros(2)

        self.nav_msg = NavSts()

        self.zero_pitch_roll = zero_pitch_roll
        self.wait_for_GPS = wait_for_GPS
        self.fix_obtained = False

        if self.wait_for_GPS:
            rospy.loginfo('%s: Will wait for GPS fix before publishing nav' % (self.name))
        else:
            rospy.loginfo('%s: Publishing nav without waiting for GPS fix' % (self.name))

        if origin is not None:
            self.origin = np.array([origin['latitude'], origin['longitude']])
            self.origin_set = True
            rospy.loginfo('%s: Setting the origin to: %s' % (self.name, self.origin))
        else:
            self.origin = np.zeros(2)  # [latitude, longitude] in radians
            self.origin_set = False
            rospy.loginfo('%s: Origin not set. Waiting for GPS fix.' % (self.name))

        self.nav_msg.origin.latitude = self.origin[0]
        self.nav_msg.origin.longitude = self.origin[1]

        self.nav_msg.global_position.latitude = self.origin[0]
        self.nav_msg.global_position.longitude = self.origin[1]

        # Distance to the centre of the Earth at given latitude assuming elipsoidal model of Earth.
        # If latitude is not known assume Earth is a sphere.
        self.geocentric_radius = frame.compute_geocentric_radius(self.origin[0])

        # Subscribers
        self.xsens_sub = rospy.Subscriber(TOPIC_XSENS, Xsens, self.handle_xsens, tcp_nodelay=True, queue_size=1)

        # Publishers
        self.nav_pub = rospy.Publisher(topic_nav, NavSts, tcp_nodelay=True, queue_size=1)
        self.nav_low_freq_pub = rospy.Publisher(topic_low_freq, NavSts, tcp_nodelay=True, queue_size=1)

        # Services
        self.srv_reset = rospy.Service(SRV_RESET_ORIGIN, BooleanService, self.handle_origin_reset)

    def loop(self):
        if self.wait_for_GPS and not self.fix_obtained:
            return

        self.nav_low_freq_pub.publish(self.nav_msg)

    def handle_xsens(self, xsens_msg):
        self.point_ll = np.array([xsens_msg.position.latitude, xsens_msg.position.longitude])

        if not self.fix_obtained and np.any(np.abs(self.point_ll) > 1):
            self.fix_obtained = True
            rospy.loginfo('%s: Got GPS fix: %s' % (self.name, self.point_ll))

        # if origin is not set yet and we are at least 1 degree away from [0, 0]
        # xsens_msg.xkf_valid: - not reliable
        if not self.origin_set and self.fix_obtained:
            self.find_geo_origin(self.point_ll, self.displacement_ne)
            self.origin_set = True
            rospy.loginfo('%s: Origin set: %s' % (self.name, self.origin))

        # prevent sending nav if GPS not fixed
        if self.wait_for_GPS and not self.fix_obtained:
            return

        # throw error if origin is too far from current point (1 degree away -> ~50km)
        if any(np.abs(self.point_ll - self.origin) > 1):
            rospy.logerr('%s: Current GPS: %s too far from origin: %s' % (self.name, self.point_ll, self.origin))
            # raise ValueError

        self.nav_msg = NavSts()
        self.nav_msg.header.stamp = rospy.Time.now()

        # global coords
        self.nav_msg.global_position.latitude = xsens_msg.position.latitude
        self.nav_msg.global_position.longitude = xsens_msg.position.longitude

        self.nav_msg.origin.latitude = self.origin[0]
        self.nav_msg.origin.longitude = self.origin[1]

        self.displacement_ne = frame.geo2ne(self.point_ll, self.origin, self.geocentric_radius)

        # pose
        self.nav_msg.position.north = self.displacement_ne[0]
        self.nav_msg.position.east = self.displacement_ne[1]
        self.nav_msg.position.depth = 0
        # self.nav_msg.position.depth = -xsens_msg.position.altitude
        self.nav_msg.altitude = xsens_msg.position.altitude

        # TODO: simplify the conversion
        # IMU returns rotation from NWU in degrees
        # orient_xyz = np.array([xsens_msg.orientation_euler.x, xsens_msg.orientation_euler.y, xsens_msg.orientation_euler.z])
        # orient_xyz = np.deg2rad(orient_xyz)
        orient_ned = np.array([xsens_msg.orientation_euler.x, xsens_msg.orientation_euler.y, xsens_msg.orientation_euler.z])
        orient_ned = np.deg2rad(orient_ned)

        # Apply rotation to get from sensor_xyz to boat_xyz rotation
        # orient_xyz = frame.wrap_pi(orient_xyz - SENSOR_ANGLE_OFFSETS)
        # orient_ned = frame.wrap_pi(orient_ned - SENSOR_ANGLE_OFFSETS)

        if self.zero_pitch_roll:
            orient_ned[0:2] = 0

        orient_rate_ned = np.array([xsens_msg.calibrated_gyroscope.x,
                                    xsens_msg.calibrated_gyroscope.y,
                                    xsens_msg.calibrated_gyroscope.z])

        # Apply rotation to get from boat_xyz to boat_ned
        # orient_ned = frame.angle_xyz2ned(orient_xyz)

        # orient_rate_ned = frame.angle_xyz2ned(orient_rate_xyz)

        # vel_ned is velocity of the sensor in NED Earth fixed reference frame
        vel_ned_earth_fixed = np.array([xsens_msg.velocity.x, xsens_msg.velocity.y, xsens_msg.velocity.z])

        # apply a rotation to get from vel_ne/ros_diamondback_ws/asv_mobility/emily_navd to vel_body
        vel_body = frame.eta_world2body(vel_ned_earth_fixed, np.zeros(3), orient_ned)

        self.nav_msg.orientation.roll = orient_ned[0]
        self.nav_msg.orientation.pitch = orient_ned[1]
        self.nav_msg.orientation.yaw = orient_ned[2]

        self.nav_msg.body_velocity.x = vel_body[0]
        self.nav_msg.body_velocity.y = vel_body[1]
        self.nav_msg.body_velocity.z = vel_body[2]
        self.nav_msg.orientation_rate.roll = orient_rate_ned[0]
        self.nav_msg.orientation_rate.pitch = orient_rate_ned[1]
        self.nav_msg.orientation_rate.yaw = orient_rate_ned[2]

        self.nav_pub.publish(self.nav_msg)

    def handle_origin_reset(self, srv):
        # set origin to where the vehicle is now
        if srv.request:
            rospy.loginfo('%s: Setting origin to: %s', self.name, self.point_ll)

            self.origin = self.point_ll
            self.displacement_ne = np.zeros(2)
            self.geocentric_radius = R_EARTH

        return BooleanServiceResponse(srv.request)

    def find_geo_origin(self, point_ll, displacement_ne):
        """Finds the origin in geo-referenced/ros_diamondback_ws/asv_mobility/emily_nav frame. The origin corresponds to (0, 0) in ned reference frame
        (where xsens was started)

        :param point_ll: current position on Earth in spherical reference frame
        :param displacement_ne: current position in ne reference (displacement from where sensor was started)
        """
        radius = frame.compute_geocentric_radius(point_ll[0])

        # NOTE: negative sign - that is because we want to look in the direction of origin
        self.origin = frame.ne2geo(-displacement_ne, point_ll, radius)
        # find the radius corresponding to the new origin
        self.geocentric_radius = frame.compute_geocentric_radius(self.origin[0])
        self.origin_set = True


if __name__ == '__main__':
    rospy.init_node('nav')
    name = rospy.get_name()

    topic_nav = rospy.get_param('~topic_nav', TOPIC_NAV)
    origin = rospy.get_param('~origin', None)
    wait_for_GPS = rospy.get_param('~wait_for_GPS', True)
    low_freq = rospy.get_param('~low_freq', LOW_FREQ)
    topic_low_freq = rospy.get_param('~topic_low_freq', TOPIC_LOW_FREQ)
    zero_pitch_roll = rospy.get_param('~zero_pitch_roll', ZERO_PITCH_ROLL)

    rospy.loginfo('%s: nav topic: %s', name, topic_nav)
    rospy.loginfo('%s: origin: %s', name, origin)
    rospy.loginfo('%s: wait for GPS: %s', name, wait_for_GPS)
    rospy.loginfo('%s: low frequency nav: %s Hz', name, low_freq)
    rospy.loginfo('%s: low frequency topic: %s', name, topic_low_freq)
    rospy.loginfo('%s: zero pitch roll: %s', name, zero_pitch_roll)

    nav = Navigation(name, topic_nav, topic_low_freq, origin, wait_for_GPS, zero_pitch_roll)
    loop_rate = rospy.Rate(low_freq)

    while not rospy.is_shutdown():
        try:
            nav.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        # except Exception as e:
        #     rospy.logfatal('%s', e)
        #     rospy.logfatal('Caught exception and dying!')
        #     sys.exit(-1)


