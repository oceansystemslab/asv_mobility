#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
np.set_printoptions(precision=3, suppress=True)

MAX_THRUST = 100
MAX_RUDDER = 100
MAX_RUDDER_ANGLE = 3*np.pi/2  # error in yaw at which maximum rudder throttle is applied
MAX_SPEED_DISTANCE = 35  # distance above which the thrust is maximum
TURNING_THR_ANGLE = np.pi/4  # error below which vehicle shoots towards the goal
STOPPING_THR = 1  # at this distance in meters thruster are switched off
SLOWDOWN_OFFSET = 2  # distance in metres at which the vehicle stops the thruster
TURNING_THRUST = 40


MAX_SPEED = 2.5
MAX_E_X_I = 0.5


# RUDDER_POS_X = 0.4  # distance from the centre of mass
# # Rudder is at angle 0 for throttle 0, and at position 0.4 for throttle 100.
# RUDDER_ANGLE_MAX = 0.4  # in radians (23 degrees).
# MAX_LINEAR_FORCE = 100  # for every linear component (not in Newtons)
# MAX_TORQUE = MAX_LINEAR_FORCE * RUDDER_POS_X * np.sin(RUDDER_ANGLE_MAX)

def wrap_pi(angle):
    return ((angle + np.pi) % (2*np.pi)) - np.pi

POINT_SHOOT = 'point_shoot'
VELOCITY_CTRL = 'velocity_ctrl'



class Controller(object):
    def __init__(self, period):
        self.pose = np.zeros(6)
        self.vel_xyz = np.zeros(6)
        self.linear_vel = 0
        self.angular_vel = 0
        self.des_pos = np.zeros(6)

        self.des_vel = np.zeros(6)
        self.des_vel_final = np.zeros(6)

        self.e_vel = 0
        self.prev_e_vel = 0
        self.ei_vel = 0
        self.ed_vel = 0

        self.kp = 0.3
        self.ki = 0.01
        self.kd = 0

        self.mode = POINT_SHOOT
        self.T = period

        self.policies = {
            POINT_SHOOT: self.eval_point_shoot
        }

    def evaluate_controller(self):
        """

        :return: a set of throttles (motor and rudder throttles) according to the control policy specified by the mode.
        """
        return self.policies[self.mode]()

    def update_nav(self, pose, **kwargs):
        # get the velocity in x-y plane, in reference frame of the boat
        if 'velocity' in kwargs.keys():
            # some transform needs to be applied
            self.vel = 0
            # self.linear_vel = kwargs['velocity'][0]
            # self.angular_vel = kwargs['velocity'][5]
        else:
            vel_xyz = (pose - self.pose)/self.T
            self.linear_vel = np.linalg.norm(vel_xyz[0:2])
            self.angular_vel = vel_xyz[5]
        self.pose = pose

    def set_mode(self, mode):
        self.mode = mode

    def request_pose(self, des_pose):
        self.des_pos = des_pose

    def request_vel(self, des_vel):
        self.des_vel = des_vel

    def eval_velocity_ctrl(self):
        vel_magnitude = np.linalg.norm(self.vel_xyz[0:2])
        vel_direction = np.arctan2(self.vel_xyz[1], self.vel_xyz[0])

        # if



    def eval_point_shoot(self):
        """Simplistic controller: first adjusts orientation (forcing forward motion) and then moves forward while attempting
        to maintain the right orientation. The vehicle has to stop the thruster before reaching the goal as there
        is no way to brake.

        :param pose: current pose [x, y, z, roll, pitch, yaw]
        :param des_pos: goal pose [x, y, z, roll, pitch, yaw] - only x and y are considered
        :return: throttle [thruster_throttle, rudder_throttle, 0,0,0,0]
            thruster_throttle is (0, 100)
            rudder_throttle is (-100, 100)
        """
        throttle = np.zeros(6)

        # depth, roll and pitch are ignored
        # the controller will attempt to get to an xy coordinate
        error_xy = self.des_pos[0:2] - self.pose[0:2]
        angle_to_goal = np.arctan2(error_xy[1], error_xy[0])

        # compute rudder throttle
        error_yaw = wrap_pi(angle_to_goal - self.pose[5])

        # map the error to the throttle, note that throttle is max for error greater than MAX_RUDDER_ANGLE radians
        # applies throttle opposite to the error
        rudder_throttle = MAX_RUDDER * -error_yaw * (1 / MAX_RUDDER_ANGLE)
        rudder_throttle = max(-100, min(100, rudder_throttle))

        distance = np.linalg.norm(error_xy)

        # compute thruster throttle
        if distance < STOPPING_THR:
            return np.zeros(6)
        # if the vehicle is pointed in the direction of the point
        elif abs(error_yaw) < TURNING_THR_ANGLE:
            thrust_throttle = self.compute_thrust_throttle(distance)
            np.clip(thrust_throttle, 0, MAX_THRUST)
        else:
            thrust_throttle = TURNING_THRUST

        throttle[0] = thrust_throttle
        throttle[1] = rudder_throttle
        return throttle

    def compute_thrust_throttle(self, distance):
        des_vel = MAX_SPEED * (distance - SLOWDOWN_OFFSET) * (1.0 / MAX_SPEED_DISTANCE)
        self.e_vel = des_vel - self.linear_vel
        self.ed_vel = self.e_vel - self.prev_e_vel
        self.prev_e_vel = self.e_vel
        self.ei_vel += self.e_vel
        self.ei_vel = np.clip(self.ei_vel, -MAX_E_X_I, MAX_E_X_I)
        thrust_throttle = MAX_THRUST * (self.kp * self.e_vel + self.ki * self.ei_vel + self.kd * self.ed_vel)
        thrust_throttle = np.clip(thrust_throttle, 0, MAX_THRUST)
        return thrust_throttle
