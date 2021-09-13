function dt_v = polyder(t_v, n_der)
    % Compute the n_der derivative of a given time vector vector
    
    % Compute the length of the array
    v_len = length(t_v);

    if (n_der > v_len - 1)
        disp("Derivative order higher than the degree of the polynomial!")
        dt_v = zeros(v_len, 1);
    else
        dt_v = t_v;

        % Derivative matrix (vlen = deg + 1)
        M = der_mat(v_len - 1);

        for i = [1 : n_der]
            dt_v = M * dt_v;
        end
    end
end

