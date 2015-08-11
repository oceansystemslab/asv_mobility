#!/usr/bin/env bash
rosservice call /emily/motors/switch "request: false"
rosservice call /emily/pilot/switch "request: false"
