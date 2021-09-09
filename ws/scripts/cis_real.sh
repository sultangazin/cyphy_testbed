#!/bin/bash
sleep 1

rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"

echo "Take off"
#rosservice call /cf2/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "3.0" "false"
rosservice call /cf2/Commander_Node/takeoff_srv "0.7" "3.0"
sleep 3

rosservice call /cf2/Commander_Node/land_srv "3.0" "[0, 0, 0]" "false"
sleep 3

rosservice call /cf2/reboot "groupMask: 0"
rosservice call /cf2/emergency "{}"
