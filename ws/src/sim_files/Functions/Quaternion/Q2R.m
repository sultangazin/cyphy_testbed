function R = Q2R(q)
    qw = q(1);
    qv = q(2:4);
    
    R = (qw^2 - qv' * qv) * eye(3) + 2.0 * qv * qv' + 2 * qw * vex(qv);
end