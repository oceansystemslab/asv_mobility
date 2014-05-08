#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

RUDDER_POS_X = 0.4  # distance from the centre of mass
# Rudder is at angle 0 for throttle 0, and at position 0.4 for throttle 100.
RUDDER_ANGLE_MAX = 0.4  # in radians (23 degrees).
MAX_LINEAR_FORCE = 100  # for every linear component (not in Newtons)
MAX_TORQUE = MAX_LINEAR_FORCE * RUDDER_POS_X * np.sin(RUDDER_ANGLE_MAX)

def wrap_pi(angle):
    return ((angle + np.pi) % (2*np.pi)) - np.pi

def allocate_force(force):
    """"Converts requested force into thruster command.
    Rudder value depends on the yawing force only.
    Thrust depends on both surge, sway and yawing forces.

    :param force: np.array size 6. [surge, sway, heave, roll, pitch, yaw]
    :return np.array size 6. [thruster_throttle, rudder_throttle, 0, 0, 0, 0]
        thruster_throttle is in range (0, 100)
        rudder_throttle is in range (-100 [turn left], 100 [turn right])
    """
    throttle = np.zeros(6)

    f_yaw = np.clip(force[5], -MAX_TORQUE, MAX_TORQUE)
    # first find the rudder_throttle first
    rudder_throttle = f_yaw/MAX_TORQUE * 100

    rudder_angle = RUDDER_ANGLE_MAX * rudder_throttle / 100
    xy_angle = np.atan2(force[1], force[0])

    # how similar is the direction of the force applied to the desired linear force? normalised to 1
    thrust_coef = np.abs(wrap_pi(xy_angle - rudder_angle))/(2*np.pi)

    return throttle