#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
np.set_printoptions(precision=3, suppress=True)


MAX_THRUST = 100
MAX_RUDDER = 100
MAX_RUDDER_ANGLE = np.pi/2  # error in yaw at which maximum rudder throttle is applied
MAX_THRUST_DISTANCE = 25  # distance above which the thrust is maximum
TURNING_THR_ANGLE = np.pi/4  # error below which vehicle shoots towards the goal
STOPPING_THR = 1  # at this distance in meters thruster are switched off
SLOWDOWN_OFFSET = 2  # distance in metres at which the vehicle stops the thruster
TURNING_THRUST = 40

# RUDDER_POS_X = 0.4  # distance from the centre of mass
# # Rudder is at angle 0 for throttle 0, and at position 0.4 for throttle 100.
# RUDDER_ANGLE_MAX = 0.4  # in radians (23 degrees).
# MAX_LINEAR_FORCE = 100  # for every linear component (not in Newtons)
# MAX_TORQUE = MAX_LINEAR_FORCE * RUDDER_POS_X * np.sin(RUDDER_ANGLE_MAX)

def wrap_pi(angle):
    return ((angle + np.pi) % (2*np.pi)) - np.pi

def point_shoot(pose, des_pos):
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
    error_xy = des_pos[0:2] - pose[0:2]
    angle_to_goal = np.arctan2(error_xy[1], error_xy[0])

    # compute rudder throttle
    error_yaw = wrap_pi(angle_to_goal - pose[5])

    # map the error to the throttle, note that throttle is max for error greater than MAX_RUDDER_ANGLE radians
    rudder_throttle = MAX_RUDDER * error_yaw * (1 / MAX_RUDDER_ANGLE)
    rudder_throttle = max(-100, min(100, rudder_throttle))

    # compute thruster throttle
    # if the vehicle is pointed in the right direction

    distance = np.linalg.norm(error_xy)

    if distance < STOPPING_THR:
        return throttle
    elif abs(error_yaw) < TURNING_THR_ANGLE:
        thrust_throttle = MAX_THRUST * (distance - SLOWDOWN_OFFSET) * (1.0 / MAX_THRUST_DISTANCE)
        thrust_throttle = max(0, min(100, thrust_throttle))
    else:
        thrust_throttle = TURNING_THRUST

    throttle[0] = thrust_throttle
    throttle[1] = rudder_throttle
    return throttle
