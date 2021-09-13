function X = vex(v)
    X = zeros(3,3);
    X(1,2) = -v(3);
    X(1,3) = v(2);
    X(2,3) = -v(1);
    
    X = X - X';
end