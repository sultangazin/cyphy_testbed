#!/bin/sh
echo "Requesting reboot"
rosservice call /cf3/reboot "{}"
