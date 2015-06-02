#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
np.set_printoptions(precision=2, suppress=True)

from emily_physics import update_jacobian

MAX_THROTTLE = 100
MAX_RUDDER = 100
MAX_RUDDER_ANGLE = 3*np.pi/2  # error in yaw at which maximum rudder throttle is applied
MAX_SPEED_DISTANCE = 35  # distance above which the thrust is maximum
TURNING_ANGLE_THR = np.pi/4  # error below which vehicle shoots towards the goal
STOPPING_THR = 1  # at this distance in meters thruster are switched off
SLOWDOWN_OFFSET = 2  # distance in metres at which the vehicle stops the thruster
TURNING_THRUST = 40  # throttle
TURNING_SPEED = 1  # m/s

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
CASCADED_PID = 'cascaded_pid'
VELOCITY_CTRL = 'velocity_ctrl'



class Controller(object):
    def __init__(self, period):
        self.mode = POINT_SHOOT
        self.dt = period

        self.policies = {
            POINT_SHOOT: self.point_shoot,
            CASCADED_PID: self.cascaded_pid
        }

        self.turning_angle_threshold = TURNING_ANGLE_THR
        self.turning_speed = TURNING_SPEED

        self.pose = np.zeros(6)
        self.body_vel = np.zeros(6)

        self.throttle = np.zeros(6)

        self.des_pose = np.zeros(6)
        self.des_vel = np.zeros(6)

        self.req_pose = np.zeros(6)
        self.req_vel = np.zeros(6)

        self.J = np.zeros((6,6))
        self.J_inv = np.zeros((6,6))

        self.min_throttle = np.array([0, -MAX_THROTTLE])
        self.max_throttle = np.array([MAX_THROTTLE, MAX_THROTTLE])

        self.kpp = np.zeros(2)
        self.kpi = np.zeros(2)
        self.kpd = np.zeros(2)
        self.kpi_limit = np.zeros(2)

        self.v_input_limit = np.zeros(2)
        self.kvp = np.zeros(2)
        self.kvi = np.zeros(2)
        self.kvd = np.zeros(2)
        self.kvi_limit = np.zeros(2)

        self.ep_p = np.zeros(2)
        self.prev_ep_p = np.zeros(2)
        self.ep_i = np.zeros(2)
        self.ep_d = np.zeros(2)

        self.ev_p = np.zeros(2)
        self.prev_ev_p = np.zeros(2)
        self.ev_i = np.zeros(2)
        self.ev_d = np.zeros(2)

    def reset_errors(self, n):
        """Sets n first entries to 0"""
        self.ep_p[0:n] = 0
        self.prev_ep_p[0:n] = 0
        self.ep_i[0:n] = 0
        self.ep_d[0:n] = 0

        self.ev_p[0:n] = 0
        self.prev_ev_p[0:n] = 0
        self.ev_i[0:n] = 0
        self.ev_d[0:n] = 0

    def evaluate_control(self):
        """

        :return: a set of throttles (motor and rudder throttles) according to the control policy specified by the mode.
        """
        return self.policies[self.mode]()

    def update_nav(self, pose, **kwargs):
        # get the body velocity
        if 'velocity' in kwargs.keys():
            self.body_vel = kwargs['velocity']
        else:
            vel_xyz = (pose - self.pose)/self.dt
            self.body_vel = np.dot(self.J_inv, vel_xyz)
            # print self.body_vel
        self.pose = np.copy(pose)

        self.J = update_jacobian(self.J, pose[3], pose[4], pose[5])
        self.J_inv = np.linalg.inv(self.J)

    def set_mode(self, mode):
        if mode != self.mode:
            self.reset_errors(2)
            self.mode = mode

    def request_pose(self, req_pose):
        self.req_pose = req_pose
        self.des_pose = req_pose

    def request_vel(self, req_vel):
        self.req_vel = req_vel
        self.des_vel = req_vel

    def cascaded_pid(self):
        self.throttle = np.zeros(6)

        # depth, roll and pitch are ignored
        # the controller will attempt to get to an xy coordinate
        e_xyz = self.des_pose - self.pose
        self.prev_ep_p = self.ep_p
        full_e_p = np.dot(self.J_inv, e_xyz)

        self.ep_p[0] = full_e_p[0]
        self.ep_p[1] = wrap_pi(np.arctan2(e_xyz[1], e_xyz[0]) - self.pose[5])

        self.ep_d = (self.ep_p - self.prev_ep_p)/self.dt

        # if destination point has been crossed then zero integral
        changed_sign = (np.sign(self.ep_p) != np.sign(self.prev_ep_p))
        self.ep_i[changed_sign] = 0

        self.ep_i = np.clip(self.ep_p + self.ep_i, -self.kpi_limit, self.kpi_limit)
        self.ep_d[1] = wrap_pi(self.ep_d[1])

        # compute the desired velocity
        self.des_vel = self.kpp * self.ep_p + self.kpi * self.ep_i + self.ep_d * self.kpd

        # ignore the translation error if bearing error is too big
        if np.abs(self.ep_p[1]) > self.turning_angle_threshold:
            self.reset_errors(1)
            self.des_vel[0] = TURNING_SPEED

        self.des_vel = np.clip(self.des_vel, -self.v_input_limit, self.v_input_limit)

        self.prev_ev_p = self.ev_p
        self.ev_p[0] = self.des_vel[0] - self.body_vel[0]
        self.ev_p[1] = wrap_pi(self.des_vel[1] - self.body_vel[5])

        self.ev_d = (self.ev_p - self.prev_ev_p)/self.dt

        # TODO: decide
        # not reasonable to keep it on velocity (maybe on yaw its ok)
        # changed_sign = (np.sign(self.ev_p) != np.sign(self.prev_ev_p))
        # self.ev_i[changed_sign] = 0

        self.ev_i = np.clip(self.ev_p + self.ev_i, -self.kvi_limit, self.kvi_limit)
        self.ep_p[1] = wrap_pi(self.ep_p[1])

        self.throttle[0:2] = self.kvp * self.ev_p + self.kvi * self.ev_i + self.ev_d * self.kvd
        # invert rudder throttle - because of how rudder rotation maps to the generated torque
        self.throttle[1] *= -1
        self.throttle[0:2] = np.clip(self.throttle[0:2], self.min_throttle, self.max_throttle)

        return self.throttle

    def point_shoot(self):
        """Simplist controller which first adjusts orientation (P control) and then moves forward while maintaining
        the right orientation. The vehicle has to stop the thruster before reaching the goal as there is no way to
        brake.
        :param pose: current pose [x, y, z, roll, pitch, yaw]
        :param des_pos: goal pose [x, y, z, roll, pitch, yaw]
        :return: throttle [thruster_throttle, rudder_throttle, 0,0,0,0]
            thruster_throttle is (0, 100)
            rudder_throttle is (-100, 100)
        """
        throttle = np.zeros(6)

        # depth, roll and pitch are ignored
        # the controller will attempt to get to an xy coordinate
        error_xy = self.req_pose[0:2] - self.pose[0:2]
        angle_to_goal = np.arctan2(error_xy[1], error_xy[0])

        # compute rudder throttle
        error_yaw = wrap_pi(angle_to_goal - self.pose[5])

        # map the error to the throttle, note that throttle is max for error greater than MAX_RUDDER_ANGLE radians
        # applies throttle opposite to the error
        rudder_throttle = MAX_RUDDER * -error_yaw * (1 / MAX_RUDDER_ANGLE)
        rudder_throttle = max(-100, min(100, rudder_throttle))

        # compute thruster throttle
        # if the vehicle is pointed in the right direction

        distance = np.linalg.norm(error_xy)

        if distance < STOPPING_THR:
            return throttle
        elif abs(error_yaw) < TURNING_ANGLE_THR:
            thrust_throttle = MAX_THROTTLE * (distance - SLOWDOWN_OFFSET) * (1.0 / MAX_SPEED_DISTANCE)
            thrust_throttle = max(0, min(100, thrust_throttle))
        else:
            thrust_throttle = TURNING_THRUST

        throttle[0] = thrust_throttle
        throttle[1] = rudder_throttle
        return throttle

    def update_gains(self, params):
        # pid parameters (position)
        self.kpp = np.array([
            params['pos_x']['kp'],
            params['pos_n']['kp'],
        ])

        self.kpd = np.array([
            params['pos_x']['kd'],
            params['pos_n']['kd'],
        ])

        self.kpi = np.array([
            params['pos_x']['ki'],
            params['pos_n']['ki'],
        ])

        self.kpi_limit = np.array([
            params['pos_x']['lim'],
            params['pos_n']['lim'],
        ])

        # pid parameters (velocity)
        self.kvp = np.array([
            params['vel_u']['kp'],
            params['vel_r']['kp'],
        ])

        self.kvd = np.array([
            params['vel_u']['kd'],
            params['vel_r']['kd'],
        ])

        self.kvi = np.array([
            params['vel_u']['ki'],
            params['vel_r']['ki'],
        ])

        self.kvi_limit = np.array([
            params['vel_u']['lim'],
            params['vel_r']['lim'],
        ])

        self.v_input_limit = np.array([
            params['vel_u']['input_lim'],
            params['vel_r']['input_lim'],
        ])

        self.turning_angle_threshold = params['turning_angle_threshold']
        self.turning_speed = params['turning_speed']

        self.reset_errors(2)

    def __str__(self):

        return """
          ep: %s
          ed: %s
          ei: %s
          evp: %s
          evd: %s
          evi: %s
          thr: %s
          pos: %s
          vel: %s
          des_vel: %s
        """ % (self.ep_p, self.ep_d, self.ep_i,
               self.ev_p, self.ev_d, self.ev_i,
               self.throttle[0:2],
               np.array([self.pose[0], self.pose[1], self.pose[5]]),
               np.array([self.body_vel[0], self.body_vel[1], self.body_vel[5]]),
               self.des_vel[0:2])
