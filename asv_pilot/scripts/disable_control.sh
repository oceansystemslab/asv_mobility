#!/usr/bin/env bash
rosservice call /motors/switch "request: false"
rosservice call /pilot/switch "request: false"
