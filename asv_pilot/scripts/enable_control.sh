#!/usr/bin/env bash
rosservice call /motors/switch "request: true"
rosservice call /pilot/switch "request: true"
