#!/bin/bash
rostopic pub /pilot/velocity_req vehicle_interface/PilotRequest "velocity: [1.5, 0.0, 0.0, 0.0, 0.0, 0.1]"
