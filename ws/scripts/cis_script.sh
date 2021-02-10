rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"

rosservice call /cf2/Commander_Node/goTo_srv "[2.5, 2.0, 0.7]" "15.0" "false"

