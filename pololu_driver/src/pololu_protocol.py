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
import serial

# Constants
OK = 0
ERROR = 1
BAUD_RATE = 2400

class PololuIF(object):
    """ Low level interface for communicating with Pololu Maestro. Sets the specific servos to specific positions.
    In order to maintain a specific position new requests have to be constantly issued at least once per second.
    """
    def __init__(self, port):
        self.port = serial.Serial(port=port,  baudrate=BAUD_RATE)

    def set_servo(self, servo_id, throttle):
        """ Sets a specific servo to a specific position using specified serial connection.

        :param servo_id: id of the servo - only 0 and 1 are used on Emily
        :param throttle: from -100 to 100.
        :return: zero if succeeded, non-zero for failure
        """
        # Clamp values
        throttle = max(-100, min(100, throttle))

        # Valid range is 0-254
        rescaled_throttle = int(0xFE * (throttle+100)/200)

        # Construct the command for moving the servo
        bud = chr(0xFF)+chr(servo_id)+chr(rescaled_throttle)

        try:
            self.port.write(bud)
            return OK
        except Exception:
            return ERROR

    def set_neutral(self, servo_id):
        return self.set_servo(servo_id, 0)

    def set_all_neutral(self):
        cum_error = 0
        for servo in [0, 1]:
            cum_error += self.set_neutral(servo)
        return cum_error