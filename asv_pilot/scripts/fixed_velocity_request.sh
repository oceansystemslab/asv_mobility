#!/bin/bash
rostopic pub -1 /emily/pilot/velocity_req vehicle_interface/PilotRequest "velocity: [1.5, 0.0, 0.0, 0.0, 0.0, 10.1]"
