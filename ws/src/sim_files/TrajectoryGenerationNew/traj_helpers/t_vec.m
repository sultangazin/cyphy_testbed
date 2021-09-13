function t_v = t_vec(t, order)
    t_v = ones(order + 1, 1);
    for i = 0 : order
        t_v(i + 1) = t^i;
    end
end