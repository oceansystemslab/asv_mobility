#!/bin/bash
 rosservice call /path/control """
command: 'path'
points:
- values: [0,0,1,0,0,0]
- values: [5.4,-9.8,1,0,0,-1.047]
- values: [150,-1,1,0,0,-1.047]
options:
- key: 'mode'
  value: 'fast'
- key: 'timeout'
  value: '3000'
"""

#- values: [1,-1,1,0,0,-1.047]
#- values: [5.4,-9.8,1,0,0,-1.047]
