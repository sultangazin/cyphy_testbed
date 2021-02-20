#!/bin/sh
echo "Requesting emergency stop"
rosservice call /cf2/emergency "{}"
