function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = zeros(4*2, n_order+1);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        T = ts(k);
        M_k(1:4, :) = getCoeffCons(0);
        M_k(5:8, :) = getCoeffCons(T);

        M = blkdiag(M, M_k);
    end
end