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

# TODO: test geo request

from __future__ import division

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
np.set_printoptions(precision=2, suppress=True)
# import sys

import frame_maths as fm

# Messages
from vehicle_interface.msg import PilotRequest, PilotStatus, ThrusterCommand
from auv_msgs.msg import NavSts
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# Constants
TOPIC_POSITION_REQUEST = '/pilot/position_req'
TOPIC_EMILY_NAV = '/nav/nav_sts'
TOPIC_NESSIE_NAV = '/modem/unpacker/nav_sts'

LOOP_RATE = 1  # Hz

CIRCLE_RADIUS = 8  # m
CIRCLE_VELOCITY = 0.3  # m
CIRCLE_TIME = 2 * np.pi * CIRCLE_RADIUS / CIRCLE_VELOCITY  # s

# point sources
SOURCE_NAV_REAL = 'nav_real'
SOURCE_NAV_SIM = 'nav_sim'
SOURCE_STATIC = 'static'

class Follower(object):
    def __init__(self, name, topic_position_request, topic_nav_follower, topic_nav_source, simulation, static_point=None):
        self.name = name

        # origin of emily
        self.follower_origin = None
        self.geo_radius = None

        # Subscribers
        self.nav_follower_sub = rospy.Subscriber(topic_nav_follower, NavSts, self.handle_follower_nav, tcp_nodelay=True, queue_size=1)
        # if we do not use a static point:
        if static_point is None:
            self.nav_source_sub = rospy.Subscriber(topic_nav_source, NavSts, self.handle_source_nav, tcp_nodelay=True, queue_size=1)

        # Publishers
        self.pos_req_pub = rospy.Publisher(topic_position_request, PilotRequest, tcp_nodelay=True, queue_size=1)

        rospy.loginfo('%s: Waiting for follower nav', self.name)
        rospy.wait_for_message(topic_nav_follower, NavSts)
        rospy.loginfo('%s: Got follower nav!', self.name)

        if static_point is not None:
            self.point_source = SOURCE_STATIC
            self.point_ll = static_point
            self.ne = fm.geo2ne(self.point_ll, self.follower_origin, self.geo_radius)
        elif simulation is True:
            self.point_source = SOURCE_NAV_SIM
            self.point_ll = None
            self.ne = None
        else:
            self.point_source = SOURCE_NAV_REAL
            self.point_ll = None
            self.ne = None

    def loop(self):
        # distance away from point in NE
        t = rospy.Time.now().to_sec() % CIRCLE_TIME
        added_circle = np.array([CIRCLE_RADIUS * np.cos(2*np.pi * t / CIRCLE_TIME),
                                 CIRCLE_RADIUS * np.sin(2*np.pi * t / CIRCLE_TIME)])

        if self.point_source == SOURCE_NAV_REAL:
            if self.point_ll is None:
                return
            self.ne = fm.geo2ne(self.point_ll, self.follower_origin, self.geo_radius)

        elif self.point_source == SOURCE_NAV_SIM:
            if self.ne is None:
                return

        # elif self.point_source == SOURCE_STATIC:
        #     pass

        req_position = np.zeros(6)
        req_position[0:2] = self.ne + added_circle

        pr = PilotRequest()
        pr.header.stamp = rospy.Time.now()
        pr.position = req_position

        self.pos_req_pub.publish(pr)

    def handle_follower_nav(self, msg):
        tmp_origin = np.array([msg.origin.latitude, msg.origin.longitude])
        if np.any(self.follower_origin != tmp_origin):
            self.follower_origin = tmp_origin
            self.geo_radius = fm.compute_geocentric_radius(self.follower_origin[0])

    def handle_source_nav(self, msg):
        if self.point_source == SOURCE_NAV_REAL:
            # do not trust nav if position is (0,0)
            if msg.global_position.latitude != 0 or msg.global_position.longitude != 0:
                self.point_ll = np.array([msg.global_position.latitude, msg.global_position.longitude])
        elif self.point_source == SOURCE_NAV_SIM:
            self.ne = np.array([msg.position.north, msg.position.east])

if __name__ == '__main__':
    rospy.init_node('nessie_follower')
    name = rospy.get_name()

    topic_position_request = rospy.get_param('~topic_position_request', TOPIC_POSITION_REQUEST)
    topic_follower_nav = rospy.get_param('~topic_emily_nav', TOPIC_EMILY_NAV)
    topic_source_nav = rospy.get_param('~topic_nessie_nav', TOPIC_NESSIE_NAV)
    static_latitude = rospy.get_param('~static_latitude', None)
    static_longitude = rospy.get_param('~static_longitude', None)
    simulation = rospy.get_param('~simulation', False)

    rospy.loginfo('%s: topic_position_request: %s', name, topic_position_request)
    rospy.loginfo('%s: topic_emily_nav: %s', name, topic_follower_nav)
    rospy.loginfo('%s: topic_nessie_nav: %s', name, topic_source_nav)
    rospy.loginfo('%s: static_latitude: %s', name, static_latitude)
    rospy.loginfo('%s: static_longitude: %s', name, static_longitude)
    rospy.loginfo('%s: simulation: %s', name, simulation)

    if static_latitude is not None and static_longitude is not None:
        static_point = np.array([static_latitude, static_longitude])
    else:
        static_point = None

    pilot = Follower(name, topic_position_request, topic_follower_nav, topic_source_nav, simulation, static_point)
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


