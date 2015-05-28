#!/bin/bash
rostopic pub /path/waypoint vehicle_interface/PathRequest "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
command: ''
points:
- values: [0.0, 30.0, 0.0, 0.0, 0.0, 0.0]
options:
- {key: '', value: ''}" 

