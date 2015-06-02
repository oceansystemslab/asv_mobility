#!/bin/bash
 rosservice call /path/control """
command: 'path'
points:
- values: [0, 0, 0, 0, 0, 0]
- values: [8, 3, 0, 0, 0, 0]
- values: [16, 0, 0, 0, 0, 0]
- values: [24, -3, 0, 0, 0, 0]
- values: [32, 0, 0, 0, 0, 0]
- values: [24, 3, 0, 0, 0, 0]
- values: [16, 0, 0, 0, 0, 0]
- values: [8, -3, 0, 0, 0, 0]
- values: [0, 0, 0, 0, 0, 0]
options:
- key: 'mode'
  value: 'fast'
- key: 'interpolation_method'
  value: 'linear'

- key: 'timeout'
  value: '300'
"""

#- values: [1,-1,1,0,0,-1.047]
#- values: [5.4,-9.8,1,0,0,-1.047]
