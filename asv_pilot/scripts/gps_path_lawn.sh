#!/usr/bin/env bash

 rostopic pub -1 /path/path_request vehicle_interface/PathRequest "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
command: ''
points:

- values: [56.384427, -4.275079, 0.0, 0.0, 0.0, 0.0]
- values: [56.384613, -4.274536, 0.0, 0.0, 0.0, 0.0]
- values: [56.384547, -4.274441, 0.0, 0.0, 0.0, 0.0]
- values: [56.384343, -4.274976, 0.0, 0.0, 0.0, 0.0]
- values: [56.384268, -4.274870, 0.0, 0.0, 0.0, 0.0]
- values: [56.384469, -4.274316, 0.0, 0.0, 0.0, 0.0]
- values: [56.384406, -4.274237, 0.0, 0.0, 0.0, 0.0]
- values: [56.384188, -4.274802, 0.0, 0.0, 0.0, 0.0]

- values: [56.384427, -4.275079, 0.0, 0.0, 0.0, 0.0]
- values: [56.384613, -4.274536, 0.0, 0.0, 0.0, 0.0]
- values: [56.384547, -4.274441, 0.0, 0.0, 0.0, 0.0]
- values: [56.384343, -4.274976, 0.0, 0.0, 0.0, 0.0]
- values: [56.384268, -4.274870, 0.0, 0.0, 0.0, 0.0]
- values: [56.384469, -4.274316, 0.0, 0.0, 0.0, 0.0]
- values: [56.384406, -4.274237, 0.0, 0.0, 0.0, 0.0]
- values: [56.384188, -4.274802, 0.0, 0.0, 0.0, 0.0]

- values: [56.384427, -4.275079, 0.0, 0.0, 0.0, 0.0]
- values: [56.384613, -4.274536, 0.0, 0.0, 0.0, 0.0]
- values: [56.384547, -4.274441, 0.0, 0.0, 0.0, 0.0]
- values: [56.384343, -4.274976, 0.0, 0.0, 0.0, 0.0]
- values: [56.384268, -4.274870, 0.0, 0.0, 0.0, 0.0]
- values: [56.384469, -4.274316, 0.0, 0.0, 0.0, 0.0]
- values: [56.384406, -4.274237, 0.0, 0.0, 0.0, 0.0]
- values: [56.384188, -4.274802, 0.0, 0.0, 0.0, 0.0]

# point A
- values: [56.385041, -4.274648, 0.0, 0.0, 0.0, 0.0]

options:
- {key: 'timeout', value: '900'}
- {key: 'path_type', value: 'gps'}"
