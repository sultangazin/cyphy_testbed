#!/bin/bash
sleep 1
sleep 1

rosservice call /cf3/Commander_Node/takeoff_srv "0.7" "3.0"
sleep 3
rosservice call /cf3/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "4.0"
sleep 4
rosservice call /cf3/Commander_Node/goTo_srv "[0.8, 0.2, 0.7]" "4.0"
sleep 4
rosservice call /cf3/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "4.0"
sleep 4
rosservice call /cf3/Commander_Node/goTo_srv "[-0.5, 0.0, 0.7]" "3.0"
sleep 3
rosservice call /cf3/Commander_Node/goTo_srv "[0.1, -0.5, 0.7]" "3.0"
sleep 3
rosservice call /landAll "{}"
sleep 3
