#!/bin/bash
rostopic pub /pilot/waypoint vehicle_interface/PilotRequest "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
priority: 0
position: [-150.0, -150.0, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
limit_velocity: [0, 0, 0, 0, 0, 0]
disable_axis: [0, 0, 0, 0, 0, 0]" 
