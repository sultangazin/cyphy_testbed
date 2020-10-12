#!/bin/sh
echo "Requesting emergency stop"
rosservice call /cf1/emergency "{}"
