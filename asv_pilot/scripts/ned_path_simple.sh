#!/usr/bin/env bash

 rostopic pub -1 /path/path_request vehicle_interface/PathRequest "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
command: ''
points:

- values: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- values: [-3.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- values: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- values: [-3.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- values: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- values: [-3.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

options:
- {key: 'timeout', value: '300'}
- {key: 'path_type', value: 'ned'}"
