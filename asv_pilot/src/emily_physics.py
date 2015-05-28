#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" The purpose of this file is to store Emily specific constants and calculations.
"""

import numpy as np

# TODO: check the constants

EMILY_MASS = 10  # (not checked) in kilograms
EMILY_LENGTH_X = 1  # (not checked) in metres
EMILY_LENGTH_y = 0.3  # (not checked) in metres

RUDDER_POSITION_X = -0.3  # (not checked) in metres with respect to the centre of mass
RUDDER_POSITION_Y = 0  # (not checked) in metres with respect to the centre of mass

# assumed that rudder position varies linearly with the throttle (roughly true)
MAX_RUDDER_ANGLE = 0.4  # (not checked) in radians - position of rudder for throttle 100
# assumed that thrust varies linearly with the throttle (untrue)
MAX_THRUST = 20  # (not checked) in Newtons
MAX_THROTTLE = 100


def compute_force(throttle):
    """Computes force acting on the boat due to thrust of the motor and position of the rudder.
    The force is in boat's reference frame.
    x is along vehicles length, y is to left of the vehicle, z is down
    roll, pitch and roll are is rotation about x, y, z respectively

    The adopted convention for axes is x along the vehicle, y to the right of the vehicle, z down.

    :param throttle: numpy array length 6, however only two first entries are used
                    [thrust_throttle, rudder_throttle, 0, 0, 0, 0]
    :return: numpy array length 6 with forces in torques in boat's reference frame
    """
    force_magnitude = MAX_THRUST * throttle[0] / MAX_THROTTLE
    force_angle = MAX_RUDDER_ANGLE * throttle[1] / MAX_THROTTLE

    # TODO: Check signs - make sure system of coordinates is ok
    lin_force = np.array([force_magnitude * np.cos(force_angle), force_magnitude * np.sin(force_angle), 0])
    torque = np.array([0, 0, force_magnitude * np.sin(force_angle) * RUDDER_POSITION_X])

    forces = np.concatenate((lin_force, torque))
    return forces


