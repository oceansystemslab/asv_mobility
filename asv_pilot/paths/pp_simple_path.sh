#!/bin/bash
 rosservice call /path/control """
command: 'path'
points:
- values: [20,0,10,0,0,0]
options:
- key: 'mode'
  value: 'simple'
- key: 'timeout'
  value: '300'
"""

