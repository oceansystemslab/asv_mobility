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

import asv_controllers as ctrl
import frame_maths as fm

# Messages
from vehicle_interface.msg import PilotRequest, PilotStatus, ThrusterCommand
from auv_msgs.msg import NavSts
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# Constants
TOPIC_THROTTLE = '/motors/throttle'
TOPIC_POSITION_REQUEST = '/pilot/position_req'
TOPIC_BODY_REQUEST = '/pilot/body_req'
TOPIC_GEO_REQUEST = '/pilot/geo_req'
TOPIC_VELOCITY_REQUEST = '/pilot/velocity_req'
TOPIC_STATUS = '/pilot/status'
TOPIC_NAV = '/nav/nav_sts'
SRV_SWITCH = '/pilot/switch'
SRV_PID_CONFIG = '/pilot/pid_config'
NAVIGATION_TIMEOUT = 5  # seconds
LOOP_RATE = 5  # Hz
SIMULATION = False
VERBOSE = False

# Status
CTRL_DISABLED = 0
CTRL_ENABLED = 1

STATUS_CTRL = {
    CTRL_DISABLED: PilotStatus.PILOT_DISABLED,
    CTRL_ENABLED: PilotStatus.PILOT_ENABLED
}

STATUS_MODE = {
    ctrl.MODE_POSITION: PilotStatus.MODE_POSITION,
    ctrl.MODE_VELOCITY: PilotStatus.MODE_VELOCITY,
    ctrl.MODE_POINT_SHOOT: 'point_shoot'
}

# TODO: move to param
SCALE_THROTTLE = 1.0

class Pilot(object):
    """Node provides an interface between control logic and ROS. This node outputs throttle commands that can be
     consumed either by pololu_driver or thruster_sim. The controller will not run if fresh navigation
     information is not available. The controller can be enabled or disabled via service.
     Generally, the asv has two degrees of freedom: surge and yaw. They are coupled - that is, the boat cannot yaw
     without thrusting forward (note that this means limited steering when slowing down).

     Different control policies:
        - point and shoot - simple P controller on position (distance to goal and orientation towards the goal).
        - cascaded pid - PID position controller outputs a desired velocity, then PID velocity controller attempts to
            achieve this velocity
        - velocity control - in progress
    """
    def __init__(self, name, topic_throttle, topic_position_request, topic_body_request, topic_geo_request,
                 topic_velocity_request, topic_nav, topic_pilot_status, srv_switch, verbose, controller_config):
        self.name = name

        # latest throttle received
        self.pose = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.vel = np.zeros(6)
        self.origin = np.zeros(2)
        self.geo_radius = fm.compute_geocentric_radius(self.origin[0])

        self.last_nav_t = 0
        self.nav_switch = False

        # TODO: set pilot_enable to False by default once tests are over
        self.pilot_enable = CTRL_DISABLED
        self.verbose = verbose

        self.controller = ctrl.Controller(1/LOOP_RATE)
        self.controller.set_mode(ctrl.MODE_POSITION)
        self.controller.update_gains(controller_config)

        # Subscribers
        self.position_sub = rospy.Subscriber(topic_position_request, PilotRequest, self.handle_pose_req, tcp_nodelay=True, queue_size=1)
        self.body_sub = rospy.Subscriber(topic_body_request, PilotRequest, self.handle_body_req, tcp_nodelay=True, queue_size=1)
        self.geo_sub = rospy.Subscriber(topic_geo_request, PilotRequest, self.handle_geo_req, tcp_nodelay=True, queue_size=1)
        self.velocity_sub = rospy.Subscriber(topic_velocity_request, PilotRequest, self.handle_vel_req, tcp_nodelay=True, queue_size=1)

        self.nav_sub = rospy.Subscriber(topic_nav, NavSts, self.handle_real_nav, tcp_nodelay=True, queue_size=1)

        # Publishers
        self.throttle_pub = rospy.Publisher(topic_throttle, ThrusterCommand, tcp_nodelay=True, queue_size=1)
        self.status_pub = rospy.Publisher(topic_pilot_status, PilotStatus, tcp_nodelay=True, queue_size=1)

        # Services
        self.srv_switch = rospy.Service(srv_switch, BooleanService, self.handle_switch)
        self.srv_pid_config = rospy.Service(SRV_PID_CONFIG, BooleanService, self.handle_pid_config)

    def loop(self):
        # if nav message is old stop the controller
        if (rospy.Time.now().to_sec() - self.last_nav_t) > NAVIGATION_TIMEOUT and self.nav_switch:
            self.nav_switch = False
            rospy.logerr('Navigation outdated')

        if self.nav_switch and self.pilot_enable:
            throttle = self.controller.evaluate_control()
            if self.verbose:
                rospy.loginfo(str(self.controller))

            thr_msg = ThrusterCommand()
            thr_msg.header.stamp = rospy.Time.now()
            thr_msg.throttle = throttle
            self.throttle_pub.publish(thr_msg)

        self.send_status()

    # TODO: test on vehicle
    def handle_real_nav(self, msg):
        try:
            pos = msg.position
            orient = msg.orientation
            vel = msg.body_velocity
            rot = msg.orientation_rate
            self.pose[0:3] = np.array([pos.north, pos.east, pos.depth])
            self.pose[3:6] = np.array([orient.roll, orient.pitch, orient.yaw])
            self.vel[0:3] = np.array([vel.x, vel.y, vel.z])
            self.vel[3:6] = np.array([rot.roll, rot.pitch, rot.yaw])

            tmp_origin = np.array([msg.origin.latitude, msg.origin.longitude])
            if np.all(self.origin != tmp_origin):
                self.origin = tmp_origin
                self.geo_radius = fm.compute_geocentric_radius(self.origin[0])

            # dt = msg.header.stamp.to_sec() - self.last_nav_t
            self.last_nav_t = msg.header.stamp.to_sec()

            self.nav_switch = True
            self.controller.update_nav(self.pose, velocity=self.vel)
        except Exception as e:
            rospy.logerr('%s', e)
            rospy.logerr('Bad navigation message format, skipping!')

    def handle_pose_req(self, msg):
        req_pose = np.array(msg.position)
        # ignore depth, pitch and roll
        if any(req_pose[2:5]):
            rospy.logwarn('Non-zero depth, pitch or roll requested. Setting those to zero.')
        req_pose[2:6] = 0
        self.controller.request_pose(req_pose)

    def handle_body_req(self, msg):
        req_pose = np.array(msg.position)
        # ignore depth, pitch and roll
        if any(req_pose[2:5]):
            rospy.logwarn('Non-zero depth, pitch or roll requested. Setting those to zero.')
        req_pose[2:6] = 0
        self.controller.request_body(req_pose)

    def handle_geo_req(self, msg):
        req_pose = np.array(msg.position)
        if any(req_pose[2:5]):
            rospy.logwarn('Non-zero depth, pitch or roll requested. Setting those to zero.')
        # ignore depth, pitch and roll
        req_pose[2:6] = 0
        req_pose[0:2] = fm.geo2ne(req_pose[0:2], self.origin, self.geo_radius)
        self.controller.request_pose(req_pose)

    def handle_vel_req(self, msg):
        req_vel = np.array(msg.velocity)
        if any(req_vel[1:5]):
            rospy.logwarn('Non-zero sway, heave, pitch or roll requested. Setting those to zero.')
        req_vel[1:5] = 0
        self.controller.request_vel(req_vel)

    def handle_switch(self, srv):
        self.pilot_enable = srv.request
        if not self.pilot_enable:
            thr_msg = ThrusterCommand()
            thr_msg.header.stamp = rospy.Time.now()
            thr_msg.throttle = np.zeros(6)
            self.throttle_pub.publish(thr_msg)
        return BooleanServiceResponse(self.pilot_enable)

    def handle_pid_config(self, srv):
        if srv.request is True:
            config = rospy.get_param('/controller', dict())
            self.controller.update_gains(config)
            rospy.logwarn("PID config reloaded")
            return BooleanServiceResponse(True)
        else:
            return BooleanServiceResponse(False)

    def send_status(self, event=None):
        ps = PilotStatus()
        ps.header.stamp = rospy.Time.now()

        ps.status = STATUS_CTRL[self.pilot_enable]
        ps.mode = STATUS_MODE[self.controller.mode]
        if self.controller.req_pose is not None:
            ps.des_pos = self.controller.req_pose.tolist()

        vel = np.zeros(6)
        vel[0:2] = self.controller.des_vel
        ps.des_vel = vel.tolist()

        self.status_pub.publish(ps)


if __name__ == '__main__':
    rospy.init_node('asv_pilot')
    name = rospy.get_name()

    topic_throttle = rospy.get_param('~topic_throttle', TOPIC_THROTTLE)
    topic_position_request = rospy.get_param('~topic_position_request', TOPIC_POSITION_REQUEST)
    topic_body_request = rospy.get_param('~topic_body_request', TOPIC_BODY_REQUEST)
    topic_geo_request = rospy.get_param('~topic_geo_request', TOPIC_GEO_REQUEST)
    topic_velocity_request = rospy.get_param('~topic_velocity_request', TOPIC_VELOCITY_REQUEST)
    topic_nav = rospy.get_param('~topic_nav', TOPIC_NAV)
    topic_pilot_status = rospy.get_param('~topic_pilot_status', TOPIC_STATUS)
    srv_switch = rospy.get_param('~srv_switch', SRV_SWITCH)
    # simulation = bool(int(rospy.get_param('~simulation', SIMULATION)))
    verbose = bool(int(rospy.get_param('~verbose', VERBOSE)))
    controller_config = rospy.get_param('~controller', dict())

    rospy.loginfo('%s: throttle topic: %s', name, topic_throttle)
    rospy.loginfo('%s: topic_position_request: %s', name, topic_position_request)
    rospy.loginfo('%s: topic_body_request: %s', name, topic_body_request)
    rospy.loginfo('%s: topic_geo_request: %s', name, topic_geo_request)
    rospy.loginfo('%s: topic_velocity_request: %s', name, topic_velocity_request)
    rospy.loginfo('%s: topic_nav: %s', name, topic_nav)
    rospy.loginfo('%s: topic_pilot_status: %s', name, topic_pilot_status)
    rospy.loginfo('%s: srv_switch: %s', name, srv_switch)
    rospy.loginfo('%s: verbose: %s', verbose)

    pilot = Pilot(name, topic_throttle, topic_position_request, topic_body_request, topic_geo_request,
                  topic_velocity_request, topic_nav, topic_pilot_status, srv_switch, verbose,
                  controller_config)
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


