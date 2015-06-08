#!/bin/bash
rostopic pub /pilot/position_req vehicle_interface/PilotRequest "position: [-20.0, -20.0, 0.0, 0.0, 0.0, 0.0]"
