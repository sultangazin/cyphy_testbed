function y = clamp(x, lb, ub)
% Clamp the input signal to a range lb/ub
    N = length(x);

    x_out = x;
    
    for (i = 1: N)
        if (x_out(i) < lb)
            x_out(i) = lb;
        end

        if (x_out(i) > ub)
            x_out(i) = ub;
        end
    end
    
    y = x_out;
end