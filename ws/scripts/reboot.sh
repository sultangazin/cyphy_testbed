#!/bin/sh
echo "Requesting reboot"
rosservice call /cf2/reboot "{}"
