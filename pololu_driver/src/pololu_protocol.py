#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial

# Constants
OK = 0
ERROR = 1
BAUD_RATE = 2400

class PololuIF(object):
    def __init__(self, port):
        self.port = serial.Serial(port=port,  baudrate=BAUD_RATE)

    def set_servo(self, servo_id, throttle):
        """ Sets a specific servo to a specific position using specified serial connection.

        :param servo_id: id of the servo - only 0 and 1 are used on Emily
        :param throttle: from -100 to 100
        :return: zero if succeeded, non-zero for failure
        """

        # Clamp values
        throttle = max(-100, min(100, throttle))

        # # Scale down the thrust
        # if servo_id == 0:
        #     throttle /= THRUSTER_SCALEDOWN

        # Valid range is 500-5500
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
        error_status = 0
        for servo in [0, 1]:
            error_status += self.set_neutral(servo)
        return error_status