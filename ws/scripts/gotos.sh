#!/bin/bash
sleep 2

# Take off
#echo "Take of with onboard controller"
rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"
rosservice call /cf2/nw_ctrl_select "sel_controller: 3"

echo "Take off"
rosservice call /cf2/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "3.0" "false"
#rosservice call /cf2/Commander_Node/takeoff_srv "0.7" "3.0"
sleep 5

#echo "Switch to external controller"
#rosservice call /cf2/nw_ctrl_select "sel_controller: 3"
#rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"
rosservice call /cf2/Commander_Node/goTo_srv "[0.7, 0.0, 0.7]" "3.0" "false"
sleep 4
rosservice call /cf2/Commander_Node/goTo_srv "[0.0, -0.7, 0.7]" "3.0" "false"
sleep 4
rosservice call /cf2/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "3.0" "false"
sleep 4 

echo -e "\nSwitching controller on the network!"
rosservice call /cf2/nw_ctrl_select "sel_controller: 4"
rosservice call /cf2/Commander_Node/goTo_srv "[0.7, 0.0, 0.7]" "3.0" "false"
sleep 4
rosservice call /cf2/Commander_Node/goTo_srv "[0.0, -0.7, 0.7]" "3.0" "false"
sleep 4
rosservice call /cf2/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "3.0" "false"
sleep 4

rosservice call /cf2/Commander_Node/land_srv "3.0" "[0, 0, 0]" "false"
sleep 3

rosservice call /cf2/reboot "groupMask: 0"
#rosservice call /cf2/stop "groupMask: 0"
