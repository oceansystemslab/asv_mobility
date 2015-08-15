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
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
import collections
np.set_printoptions(precision=2, suppress=True)

import frame_maths as fm

# Messages
from vehicle_interface.msg import PilotRequest, PilotStatus, PathRequest
from auv_msgs.msg import NavSts
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# Constants
TOPIC_PATH_REQUEST = '/path/path_request'
TOPIC_POSITION_REQUEST = '/pilot/position_req'
TOPIC_BODY_REQUEST = '/pilot/body_req'
TOPIC_NAV = '/nav/nav_sts'
SRV_SWITCH = '/path/switch'
SRV_CLEAR_PATH = '/path/clear'
LOOP_RATE = 5  # Hz
VERBOSE = False

GPS_PATH = 'gps'
NED_PATH = 'ned'

TOLERANCE_DISTANCE = 2.5


class PathFollower(object):
    def __init__(self, name, topic_path, topic_position_request, topic_body_request, topic_nav, srv_switch, srv_clear_path):
        self.name = name

        self.path_active = False
        self.points = None
        self.index = None
        self.t_start = None
        self.timeout = None
        # is the path NED or GPS based?
        self.path_type = None

        # latest throttle received
        self.pose = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.origin = np.zeros(2)
        self.geo_radius = fm.compute_geocentric_radius(self.origin[0])

        self.switch = True

        # Subscribers
        self.nav_sub = rospy.Subscriber(topic_nav, NavSts, self.handle_nav, tcp_nodelay=True, queue_size=1)
        self.path_sub = rospy.Subscriber(topic_path, PathRequest, self.handle_path, tcp_nodelay=True, queue_size=1)

        # Publishers
        self.position_pub = rospy.Publisher(topic_position_request, PilotRequest, tcp_nodelay=True, queue_size=1)
        self.body_pub = rospy.Publisher(topic_body_request, PilotRequest, tcp_nodelay=True, queue_size=1)

        # Services
        self.srv_switch = rospy.Service(srv_switch, BooleanService, self.handle_switch)
        self.srv_clear_path = rospy.Service(srv_clear_path, BooleanService, self.handle_clear_path)

    def loop(self):
        if self.switch is False:
            return

        if self.points is None:
            return

        if self.index >= len(self.points):
            rospy.loginfo('%s: Path complete!', self.name)
            self.clear_path()
            return

        if rospy.Time.now().to_sec() - self.t_start > self.timeout:
            rospy.logwarn('%s: Timeout! Clearing the path!', self.name)
            self.clear_path()
            self.send_zero_req()
            return

        des_point = self.points[self.index]

        if self.point_reached(des_point):
            rospy.loginfo('%s: Point reached: %s.', self.name, des_point[0:2])
            self.index += 1
        else:
            pr = PilotRequest()
            pr.header.stamp = rospy.Time.now()
            pr.position = des_point
            self.position_pub.publish(pr)

    def handle_path(self, msg):
        if self.path_active is True:
            rospy.logwarn('%s: Previous path not complete. Ignoring the new one!', self.name)
            rospy.logwarn('%s: Send clear service to cancel the current path.', self.name)
            return

        if len(msg.points) == 0:
            rospy.logerr('%s: Empty path received. Ignoring!', self.name)
            return

        options = {}
        for option in msg.options:
            options.update({option.key: option.value})

        try:
            path_type = options['path_type']
            timeout = float(options['timeout'])
        except KeyError:
            rospy.logerr('%s: timeout or path_type not specified! Ignoring the request!', self.name)
            return

        path_points = np.zeros((len(msg.points), 6))
        for i, point in enumerate(msg.points):
            if path_type == NED_PATH:
                path_points[i] = np.array(point.values)
            elif path_type == GPS_PATH:
                path_points[i][0:2] = fm.geo2ne(point.values[0:2], self.origin, self.geo_radius)

        # zero everything except NE
        path_points[:, 2:6] = 0

        self.points = path_points

        self.index = 0
        self.timeout = timeout
        self.path_type = path_type
        self.t_start = rospy.Time.now().to_sec()
        self.path_active = True
        rospy.loginfo('%s: Starting new path!', self.name)

    def handle_nav(self, msg):
        pos = msg.position
        orient = msg.orientation
        self.pose[0:3] = np.array([pos.north, pos.east, pos.depth])
        self.pose[3:6] = np.array([orient.roll, orient.pitch, orient.yaw])

        tmp_origin = np.array([msg.origin.latitude, msg.origin.longitude])
        if np.all(self.origin != tmp_origin):
            self.origin = tmp_origin
            self.geo_radius = fm.compute_geocentric_radius(self.origin[0])
            rospy.logwarn('%s: Changed origin during ned path. Discarding current path.', self.name)
            self.clear_path()
            self.send_zero_req()

    def handle_switch(self, srv):
        self.switch = srv.request
        return BooleanServiceResponse(srv.request)

    def handle_clear_path(self, srv):
        if srv.request:
            self.clear_path()
            self.send_zero_req()
        return BooleanServiceResponse(srv.request)

    def clear_path(self):
        self.path_active = False
        self.points = None
        self.index = None
        self.t_start = None
        self.timeout = None
        # is the path NED or GPS based?
        self.path_type = None

    def send_zero_req(self):
        br = PilotRequest()
        br.header.stamp = rospy.Time.now()
        br.position = np.zeros(6).tolist()
        rospy.loginfo('%s: Sending stay request at: %s', self.name, self.pose[0:2])
        self.body_pub.publish(br)

    def point_reached(self, point):
        return np.linalg.norm(self.pose[0:2] - point[0:2]) < TOLERANCE_DISTANCE


if __name__ == '__main__':
    rospy.init_node('asv_pilot')
    name = rospy.get_name()

    topic_path = rospy.get_param('~topic_path_request', TOPIC_PATH_REQUEST)
    topic_position_request = rospy.get_param('~topic_position_request', TOPIC_POSITION_REQUEST)
    topic_body_request = rospy.get_param('~topic_body_request', TOPIC_BODY_REQUEST)
    topic_nav = rospy.get_param('~topic_nav', TOPIC_NAV)
    srv_switch = rospy.get_param('~srv_switch', SRV_SWITCH)
    srv_clear_path = rospy.get_param('~srv_clear', SRV_CLEAR_PATH)

    rospy.loginfo('%s: topic_path: %s', name, topic_path)
    rospy.loginfo('%s: topic_position_request: %s', name, topic_position_request)
    rospy.loginfo('%s: topic_body_request: %s', name, topic_body_request)
    rospy.loginfo('%s: topic_nav: %s', name, topic_nav)
    rospy.loginfo('%s: srv_switch: %s', name, srv_switch)
    rospy.loginfo('%s: srv_clear_path: %s', name, srv_clear_path)

    pf = PathFollower(name, topic_path, topic_position_request, topic_body_request, topic_nav, srv_switch, srv_clear_path)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            pf.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        # except Exception as e:
        #     rospy.logfatal('%s', e)
        #     rospy.logfatal('Caught exception and dying!')
        #     sys.exit(-1)


