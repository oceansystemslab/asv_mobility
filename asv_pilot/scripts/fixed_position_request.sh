#!/bin/bash
rostopic pub -1 /pilot/position_req vehicle_interface/PilotRequest "position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
