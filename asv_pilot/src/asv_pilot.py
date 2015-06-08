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
np.set_printoptions(precision=2, suppress=True)
# import sys

import asv_controllers as ctrl

# Messages
from vehicle_interface.msg import PilotRequest, ThrusterCommand
from auv_msgs.msg import NavSts
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# Constants
TOPIC_THROTTLE = '/motors/throttle'
TOPIC_REQUEST = '/pilot/position_req'
TOPIC_NAV = '/nav/nav_sts'
SRV_SWITCH = '/pilot/switch'
SRV_PID_CONFIG = '/pilot/pid_config'
NAVIGATION_TIMEOUT = 5  # seconds
LOOP_RATE = 10  # Hz
SIMULATION = False

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
    def __init__(self, name, topic_throttle, topic_request, simulation, controller_config):
        self.name = name

        # latest throttle received
        self.pose = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.vel = np.zeros(6)
        self.des_pose = np.zeros(6)

        self.last_nav_t = 0
        self.nav_switch = False

        # TODO: pilot_enable to False by default once tests are over
        self.pilot_enable = True
        self.simulation = simulation

        self.controller = ctrl.Controller(1/LOOP_RATE)
        self.controller.set_mode(ctrl.CASCADED_PID)
        self.controller.update_gains(controller_config)

        # Subscribers
        self.waypoint_sub = rospy.Subscriber(topic_request, PilotRequest, self.handle_waypoint)
        if self.simulation:
            self.nav_sub = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_sim_nav)
            rospy.loginfo('Using NavSts from simulation (simulation).')
        else:
            self.nav_sub = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_real_nav)
            rospy.loginfo('Using NavSts from vehicle (real run).')

        # Publishers
        self.throttle_pub = rospy.Publisher(topic_throttle, ThrusterCommand)

        # Services
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)
        self.srv_pid_config = rospy.Service(SRV_PID_CONFIG, BooleanService, self.handle_pid_config)

    def loop(self):
        throttle = np.zeros(6)
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_nav_t) > NAVIGATION_TIMEOUT and self.nav_switch:
            self.nav_switch = False
            rospy.logerr('Odometry outdated')

        if self.nav_switch and self.pilot_enable:
            self.controller.request_pose(self.des_pose)
            throttle = self.controller.evaluate_control()
            rospy.loginfo(str(self.controller))

        throttle_msg = ThrusterCommand()
        throttle_msg.header.stamp = rospy.Time().now()
        throttle_msg.throttle = throttle
        self.throttle_pub.publish(throttle_msg)

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
            dt = msg.header.stamp.to_sec() - self.last_nav_t
            self.last_nav_t = msg.header.stamp.to_sec()
            self.nav_switch = True
            self.controller.update_nav(self.pose, dt, velocity=self.vel)
        except Exception as e:
            rospy.logerr('%s', e)
            rospy.logerr('Bad navigation message format, skipping!')

    def handle_sim_nav(self, msg):
        try:
            pos = msg.position
            orient = msg.orientation
            vel = msg.body_velocity
            rot = msg.orientation_rate
            self.pose[0:3] = np.array([pos.north, pos.east, pos.depth])
            self.pose[3:6] = np.array([orient.roll, orient.pitch, orient.yaw])
            self.vel[0:3] = np.array([vel.x, vel.y, vel.z])
            self.vel[3:6] = np.array([rot.roll, rot.pitch, rot.yaw])
            dt = msg.header.stamp.to_sec() - self.last_nav_t
            self.last_nav_t = msg.header.stamp.to_sec()
            self.nav_switch = True
            self.controller.update_nav(self.pose, dt)
        except Exception as e:
            rospy.logerr('%s', e)
            rospy.logerr('Bad navigation message format, skipping!')

    def handle_waypoint(self, msg):
        try:
            self.des_pose = np.array(msg.position)
            # ignore depth, pitch and roll
            if any(self.des_pose[2:5]):
                rospy.logwarn('Non-zero depth, pitch or roll requested.')
            self.des_pose[2:5] = 0
        except Exception as e:
            rospy.logerr('%s', e)
            rospy.logerr('Bad waypoint message format, skipping!')

    def handle_switch(self, srv):
        self.pilot_enable = srv.request
        return BooleanServiceResponse(True)

    def handle_pid_config(self, srv):
        config = rospy.get_param('~controller', dict())
        self.controller.update_gains(config)
        rospy.loginfo("PID config reloaded")
        return BooleanServiceResponse(True)

if __name__ == '__main__':
    rospy.init_node('asv_pilot')
    name = rospy.get_name()

    topic_throttle = rospy.get_param('~topic_throttle', TOPIC_THROTTLE)
    topic_request = rospy.get_param('~topic_request', TOPIC_REQUEST)
    simulation = bool(int(rospy.get_param('~simulation', SIMULATION)))
    controller_config = rospy.get_param('~controller', dict())

    print controller_config

    rospy.loginfo('throttle topic: %s', topic_throttle)
    rospy.loginfo('simulation: %s', simulation)

    pilot = Pilot(name, topic_throttle, topic_request, simulation, controller_config)
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


