#!/usr/bin/env bash

rosparam load ../conf/pid_sim.yaml
rosservice call /pilot/pid_config "request: true"