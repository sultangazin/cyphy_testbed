function [m1, m2, m3, m4] = ctrl_cmd2motors(thrust, rpy)
    % Function that maps from rpy/thrust to motor commands
    thrust = clamp(thrust, 0, 65535);
    rpy = clamp(rpy, -32000, 32000);
   
    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);
    
    r = roll / 2.0; 
    p = pitch / 2.0;
    
    m1 = clamp(thrust - r - p + yaw, 0, 65535);
    m2 = clamp(thrust - r + p - yaw, 0, 65535);
    m3 = clamp(thrust + r + p + yaw, 0, 65535);
    m4 = clamp(thrust + r - p - yaw, 0, 65535);
end

