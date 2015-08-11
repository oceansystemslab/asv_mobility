#!/usr/bin/env bash

rosparam load ../conf/pid_real.yaml
rosservice call /pilot/pid_config "request: true"