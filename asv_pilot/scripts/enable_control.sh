#!/usr/bin/env bash
rosservice call /emily/motors/switch "request: true"
rosservice call /emily/pilot/switch "request: true"
