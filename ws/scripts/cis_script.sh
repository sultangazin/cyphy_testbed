rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"

rosservice call /cf2/Commander_Node/goTo_srv "[0.0, 0.0, 0.7]" "3.0" "false"

