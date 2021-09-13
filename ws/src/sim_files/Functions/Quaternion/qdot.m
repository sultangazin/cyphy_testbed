function y = qdot(AngVel, quat)
% Compute the quaternion derivative given the 
% quaternion and the angular velocity.
    Q = Qq(quat);
    
    AngVel_q = zeros(4, 1);
    
    AngVel_q(2:4) = AngVel;
    
    y = 0.5 * Q * AngVel_q;
end