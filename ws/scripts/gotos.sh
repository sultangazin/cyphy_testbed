#!/bin/bash
sleep 1
sleep 1


rosservice call /cf3/Commander_Node/ctrl_offboard_srv "offboard_active: true"

rosservice call /cf3/Commander_Node/takeoff_srv "0.7" "3.0"
sleep 6

rosservice call /cf3/stop "groupMask: 0"

#rosservice call /cf3/Commander_Node/goTo_srv "[0.5, 0.0, 0.7]" "4.0"
#sleep 4
#rosservice call /cf3/Commander_Node/goTo_srv "[0.5, 0.5, 0.7]" "4.0"
#sleep 4

## This is outside the carpet 
#rosservice call /cf3/Commander_Node/goTo_srv "[1.1, 1.7, 0.7]" "5.0"
#sleep 5
## This is outside the carpet 
#rosservice call /cf3/Commander_Node/goTo_srv "[1.1, 1.8, 0.7]" "4.0"
#sleep 4
#
#rosservice call /cf3/Commander_Node/goTo_srv "[1.1, 1.2, 0.7]" "4.0"
#sleep 4
#
#rosservice call /cf3/Commander_Node/goTo_srv "[0.3, 0.5, 0.7]" "4.0"
#sleep 4
#rosservice call /cf3/Commander_Node/land_srv "3.0" "[0.3, 0.5, 0.0]"
#sleep 3
