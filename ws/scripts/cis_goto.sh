#!/bin/bash
sleep 1

rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"

echo "Take off"
rosservice call /cf2/Commander_Node/goTo_srv "[-1.0, -0.8, 0.3]" "3.0" "false"
sleep 3

rosservice call /cf2/Commander_Node/goTo_srv "[0.8, 0.8, 0.3]" "10.0" "false"
sleep 10

rosservice call /cf2/Commander_Node/land_srv "3.0" "[0.8, 0.8, 0]" "false"
sleep 4

rosservice call /cf2/reboot "groupMask: 0"
rosservice call /cf2/emergency "{}"
