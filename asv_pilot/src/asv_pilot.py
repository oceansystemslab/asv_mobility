#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import roslib
roslib.load_manifest('asv_pilot')

import rospy
import numpy as np
np.set_printoptions(precision=1, suppress=True)
# import sys


from transformations import euler_from_quaternion
import controllers as ctrl

# Messages
from vehicle_interface.msg import PilotRequest, ThrusterCommand
from nav_msgs.msg import Odometry
from auv_msgs.msg import NavSts
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# Constants
TOPIC_THROTTLE = '/motors/throttle'
TOPIC_ODOMETRY = '/nav/odometry'
TOPIC_REQUEST = '/pilot/position_req'
TOPIC_NAV = '/nav/nav_sts'
SRV_SWITCH = '/pilot/switch'
SRV_PID_CONFIG = '/pilot/pid_config'
ODOMETRY_TIMEOUT = 5  # seconds
LOOP_RATE = 10  # Hz
SIMULATION = False

class Pilot(object):
    """Node provides an interface between control logic and ROS. This node outputs throttle commands that can be
     consumed either by pololu_driver or thruster_sim. The controller will not run if fresh odometry
     information is not available. The controller can be enabled or disabled via service.
     Generally, the asv has two degrees of freedom: surge and yaw. They are coupled - that is the boat cannot yaw
     without moving forward.
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

        self.last_odometry_t = 0
        self.odometry_switch = False
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
            self.odometry_sub = rospy.Subscriber(TOPIC_ODOMETRY, Odometry, self.handle_odometry)
            # self.nav_sub = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_real_nav)
            rospy.loginfo('Using NavSts from vehicle (real run).')

        # Publishers
        self.throttle_pub = rospy.Publisher(topic_throttle, ThrusterCommand)

        # Services
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)
        self.srv_pid_config = rospy.Service(SRV_PID_CONFIG, BooleanService, self.handle_pid_config)

    def loop(self):
        # if message is old and throttle is non-zero then set to zero
        if (rospy.Time.now().to_sec() - self.last_odometry_t) > ODOMETRY_TIMEOUT and self.odometry_switch:
            self.odometry_switch = False
            rospy.logerr('Odometry outdated')

        if self.odometry_switch and self.pilot_enable:
            # self.controller.update_nav(self.pose, velocity=self.vel)
            self.controller.update_nav(self.pose)
            self.controller.request_pose(self.des_pose)
            throttle = self.controller.evaluate_control()
            rospy.loginfo(str(self.controller))
            # rospy.loginfo('pose: %s des pose: %s throttles: %s', self.pose, self.des_pose[0:2], throttle[0:2])
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
            # TODO: add velocity
            # self.vel =
            self.last_odometry_t = msg.header.stamp.to_sec()
            self.odometry_switch = True
        except Exception as e:
            rospy.logerr('%s', e)
            rospy.logerr('Bad odometry message format, skipping!')

    # def handle_real_nav(self, msg):
    #     try:
    #         self.pose[0:2] = geo2xyz(msg.global_position.latitude, msg.global_position.longitude, self.origin)
    #         # self.pose[2] = msg.altitude
    #         self.pose[2] = 0
    #         orient = msg.orientation
    #         vel = msg.body_velocity
    #         rot = msg.orientation_rate
    #         self.pose[3:6] = np.deg2rad(np.array([orient.roll, orient.pitch, orient.yaw]))
    #         self.vel[0:3] = np.array([vel.x, vel.y, vel.z])
    #         self.vel[3:6] = np.array([rot.roll, rot.pitch, rot.yaw])
    #
    #         self.last_odometry_t = msg.header.stamp.to_sec()
    #         self.odometry_switch = True
    #     except Exception as e:
    #         rospy.logerr('%s', e)
    #         rospy.logerr('Bad odometry message format, skipping!')

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
            self.last_odometry_t = msg.header.stamp.to_sec()
            self.odometry_switch = True
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


