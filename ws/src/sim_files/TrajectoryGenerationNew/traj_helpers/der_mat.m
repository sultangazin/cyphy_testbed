function M = der_mat(deg)
    N = deg + 1;
    M = zeros(N);
    
    for (i = 1 : N - 1)
        M(i + 1, i) = i;
    end
end