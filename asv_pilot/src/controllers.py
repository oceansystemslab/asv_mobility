#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
np.set_printoptions(precision=3, suppress=True)

RUDDER_POS_X = 0.4  # distance from the centre of mass
# Rudder is at angle 0 for throttle 0, and at position 0.4 for throttle 100.
RUDDER_ANGLE_MAX = 0.4  # in radians (23 degrees).
MAX_LINEAR_FORCE = 100  # for every linear component (not in Newtons)
MAX_TORQUE = MAX_LINEAR_FORCE * RUDDER_POS_X * np.sin(RUDDER_ANGLE_MAX)

def wrap_pi(angle):
    return ((angle + np.pi) % (2*np.pi)) - np.pi

def point_shoot(pose, des_pos):
    """

    :param pose: current pose [x, y, z, roll, pitch, yaw]
    :param des_pos: goal pose [x, y, z, roll, pitch, yaw]
    :return: throttle [thruster_throttle, rudder_throttle, 0,0,0,0]
        thruster_throttle is (0, 100)
        rudder_throttle is (-100, 100)
    """
    throttle = np.zeros(6)

    # depth, roll and pitch are ignored
    pose[2:5] = 0
    # the controller will attempt to get to an xy coordinate
    des_pos[2:6] = 0

    error_xy = des_pos[0:2] - pose[0:2]
    angle_to_goal = np.arctan2(error_xy[1], error_xy[2])

    error_yaw = wrap_pi(angle_to_goal - pose[5])


    rudder_throttle = f_yaw/MAX_TORQUE * 100

    rudder_angle = RUDDER_ANGLE_MAX * rudder_throttle / 100
    xy_angle = np.atan2(force[1], force[0])

    # how similar is the direction of the force applied to the desired linear force? normalised to 1
    thrust_coef = np.abs(wrap_pi(xy_angle - rudder_angle))/(2*np.pi)

    return throttle