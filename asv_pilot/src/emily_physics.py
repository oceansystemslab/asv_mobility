#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" The purpose of this file is to store Emily specific constants and calculations.
"""

import numpy as np

EMILY_MASS = 10  # (not checked) in kilograms
EMILY_LENGTH_X = 1  # (not checked) in metres
EMILY_LENGTH_y = 0.3  # (not checked) in metres

RUDDER_POSITION_X = -0.3  # (not checked) in metres with respect to the centre of mass
RUDDER_POSITION_Y = 0  # (not checked) in metres with respect to the centre of mass

# assumed that rudder position varies linearly with the throttle (roughly true)
MAX_RUDDER_ANGLE = 0.4  # (not checked) in radians - position of rudder for throttle 100
# assumed that thrust varies linearly with the throttle (untrue)
MAX_THRUST = 150  # (not checked) in Newtons
MAX_THROTTLE = 100


def compute_TAM(rudder_angle):
    TAM = np.array([
        [ np.cos(rudder_angle) ], # surge
        [ np.sin(rudder_angle) ], # sway
        [ 0 ],  # heave
        [ 0 ],  # roll
        [ 0 ],  # pitch
        [ np.sin(rudder_angle) * RUDDER_POSITION_X ]  # yaw
    ])
    return TAM

def compute_body_force(throttle):
    """Computes force acting on the boat due to thrust of the motor and position of the rudder.
    The force is in boat's reference frame.
    x is along vehicles length, y is to left of the vehicle, z is down
    roll, pitch and roll are is rotation about x, y, z respectively

    The adopted convention for axes is x along the vehicle, y to the right of the vehicle, z down.

    :param throttle: numpy array length 6, however only two first entries are used
                    [thrust_throttle, rudder_throttle, 0, 0, 0, 0]
    :return: numpy array length 6 with forces and torques in boat's reference frame
    """
    force_magnitude = MAX_THRUST * throttle[0] / MAX_THROTTLE
    # map to rudder angle
    rudder_angle = MAX_RUDDER_ANGLE * throttle[1] / MAX_THROTTLE

    # lin_force = np.array([force_magnitude * np.cos(rudder_angle), force_magnitude * np.sin(rudder_angle), 0])
    # torque = np.array([0, 0, force_magnitude * np.sin(rudder_angle) * RUDDER_POSITION_X])
    # forces = np.concatenate((lin_force, torque))

    forces = np.dot(compute_TAM(rudder_angle), np.array([force_magnitude])).flatten()

    return forces



#pythran export compute_jacobian(float, float, float)
def compute_jacobian(phi, theta, psi):
    """This functions computes the jacobian matrix used for converting body-frame to earth-frame coordinates.

    :param phi: pitch angle (k)
    :param theta: roll angle (m)
    :param psi: yaw angle (n)
    :return: J matrix (6x6)
    """
    J = np.zeros((6,6))

    # jacobian one
    J[0, 0] = np.cos(theta) * np.cos(psi)
    J[0, 1] = np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi)
    J[0, 2] = np.sin(psi) * np.sin(phi) + np.cos(psi) * np.cos(phi) * np.sin(theta)

    J[1, 0] = np.cos(theta) * np.sin(psi)
    J[1, 1] = np.cos(psi) * np.cos(phi) + np.sin(phi) * np.sin(theta) * np.sin(psi)
    J[1, 2] = np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)

    J[2, 0] = -np.sin(theta)
    J[2, 1] = np.cos(theta) * np.sin(phi)
    J[2, 2] = np.cos(theta) * np.cos(phi)

    # jacobian two
    J[3, 3] = 1.0
    J[3, 4] = np.sin(phi) * np.tan(theta)
    J[3, 5] = np.cos(phi) * np.tan(theta)

    J[4, 3] = 0.0
    J[4, 4] = np.cos(phi)
    J[4, 5] = -np.sin(phi)

    J[5, 3] = 0.0
    J[5, 4] = np.sin(phi) / np.cos(theta)
    J[5, 5] = np.cos(phi) / np.cos(theta)

    return J


#pythran export update_jacobian(float[][], float, float, float)
def update_jacobian(J, phi, theta, psi):
    """This functions computes the jacobian matrix used for converting body-frame to earth-frame coordinates.

    :param J: jacobian matrix (6x6)
    :param phi: pitch angle (k)
    :param theta: roll angle (m)
    :param psi: yaw angle (n)
    :return: J matrix (6x6)
    """
    J = np.zeros((6,6))

    # jacobian one
    J[0, 0] = np.cos(theta) * np.cos(psi)
    J[0, 1] = np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi)
    J[0, 2] = np.sin(psi) * np.sin(phi) + np.cos(psi) * np.cos(phi) * np.sin(theta)

    J[1, 0] = np.cos(theta) * np.sin(psi)
    J[1, 1] = np.cos(psi) * np.cos(phi) + np.sin(phi) * np.sin(theta) * np.sin(psi)
    J[1, 2] = np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)

    J[2, 0] = -np.sin(theta)
    J[2, 1] = np.cos(theta) * np.sin(phi)
    J[2, 2] = np.cos(theta) * np.cos(phi)

    # jacobian two
    J[3, 3] = 1.0
    J[3, 4] = np.sin(phi) * np.tan(theta)
    J[3, 5] = np.cos(phi) * np.tan(theta)

    J[4, 3] = 0.0
    J[4, 4] = np.cos(phi)
    J[4, 5] = -np.sin(phi)

    J[5, 3] = 0.0
    J[5, 4] = np.sin(phi) / np.cos(theta)
    J[5, 5] = np.cos(phi) / np.cos(theta)

    return J
