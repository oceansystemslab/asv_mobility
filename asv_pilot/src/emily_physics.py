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
