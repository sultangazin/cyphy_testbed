#!/bin/bash
sleep 1
sleep 1

# Take off
echo "Take of with onboard controller"
#rosservice call /cf3/Commander_Node/takeoff_srv "0.5" "3.0"
sleep 3

# Switch to external controller
echo "Switch to external controller"
rosservice call /cf3/Commander_Node/ctrl_offboard_srv "offboard_active: true"

rosservice call /cf3/Commander_Node/takeoff_srv "0.7" "3.0"
sleep 3

#echo "Move +x"
#rosservice call /cf3/Commander_Node/goTo_srv "[0.2, 0.0, 0.5]" "3.0" "false"
#sleep 3
#echo "Move -x"
#rosservice call /cf3/Commander_Node/goTo_srv "[-0.2, 0.0, 0.5]" "3.0" "false"
#sleep 3
#echo "Move +y"
#rosservice call /cf3/Commander_Node/goTo_srv "[0.0, 0.2, 0.5]" "3.0" "false"
#sleep 3
#echo "Move -y"
#rosservice call /cf3/Commander_Node/goTo_srv "[0.0, -0.2, 0.5]" "3.0" "false"
#sleep 3

#echo "Land with internal controller"
#rosservice call /cf3/Commander_Node/land_srv "3.0" "[0, 0, 0]" "true"
#sleep 3

# Move

# Switch back to internal controller
#rosservice call /cf3/Commander_Node/ctrl_offboard_srv "offboard_active: false"

# Move
#rosservice call /cf3/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "3.0" "false"
#sleep 3

## This is outside the carpet 
#rosservice call /cf3/Commander_Node/goTo_srv "[1.1, 1.7, 0.7]" "5.0" "false"
#sleep 5
## This is outside the carpet 
#rosservice call /cf3/Commander_Node/goTo_srv "[1.1, 1.8, 0.7]" "4.0" "false"
#sleep 4
#
#rosservice call /cf3/Commander_Node/goTo_srv "[1.1, 1.2, 0.7]" "4.0" "false"
#sleep 4
#
#rosservice call /cf3/Commander_Node/goTo_srv "[0.3, 0.5, 0.7]" "4.0" "false"
#sleep 4
rosservice call /cf3/Commander_Node/land_srv "3.0" "[0, 0, 0]" "true"
sleep 3

#rosservice call /cf3/reboot "groupMask: 0"
#rosservice call /cf3/stop "groupMask: 0"
