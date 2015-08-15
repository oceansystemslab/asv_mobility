#!/usr/bin/env bash

 rostopic pub -1 /emily/path/path_request vehicle_interface/PathRequest "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
command: ''
points:
- values: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- values: [10.0, 10.0, 0.0, 0.0, 0.0, 0.0]
- values: [-10.0, 10.0, 0.0, 0.0, 0.0, 0.0]
- values: [-10.0, -10.0, 0.0, 0.0, 0.0, 0.0]
- values: [10.0, -10.0, 0.0, 0.0, 0.0, 0.0]
- values: [10.0, 10.0, 0.0, 0.0, 0.0, 0.0]
- values: [100.0, 100.0, 0.0, 0.0, 0.0, 0.0]
options:
- {key: 'timeout', value: '300'}
- {key: 'path_type', value: 'ned'}"
