#!/usr/bin/env bash

# usage
print_usage() {
    echo "Usage: $(basename $0) <LAT> <LONG>"
    echo "Send the body request over acoustic link."
    echo ""
    echo "Mandatory arguments:"
    echo "  <N>: translation in vehicle's North in metres"
    echo "  <E>: translation in vehicle's East in metres"
    echo ""
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

# vars
LAT=$1
LONG=$2

rostopic pub -1 /pilot/geo_req vehicle_interface/PilotRequest "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
priority: 0
position: [${LAT}, ${LONG}, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
limit_velocity: [0, 0, 0, 0, 0, 0]
disable_axis: [0, 0, 0, 0, 0, 0]"

