function v = vee(X) 
    v = zeros(3,1);
    
    v(1) = -X(2, 3);
    v(2) =  X(1, 3);
    v(3) = -X(1, 2);
end