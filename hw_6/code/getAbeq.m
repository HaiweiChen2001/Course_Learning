function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    d_order = (n_order + 1)/2;
    %#####################################################
    % STEP 2.1 起点的状态约束
    Aeq_start = zeros(d_order, n_all_poly);
    Aeq_start(1:d_order, 1:n_order+1) = getCoeffCons(0)*getM(n_order);
    beq_start =  start_cond';
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    t = ts(end);
    Aeq_end = zeros(d_order, n_all_poly);
    Aeq_end(1:d_order, end-n_order:end) = getCoeffCons(t)*getM(n_order);
    beq_end = end_cond';
    
    %#####################################################
    % STEP 2.3 连续性约束
    Aeq_con = zeros((n_seg-1)*d_order, n_all_poly);
    beq_con = zeros((n_seg-1)*d_order, 1);
    for n = 0:n_seg-1-1
        Aeq_con(1+d_order*n:d_order+d_order*n, 1+n*(n_order+1):n_order+1+n*(n_order+1)) = getCoeffCons(ts(n+1))*getM(n_order);
        Aeq_con(1+d_order*n:d_order+d_order*n, 1+(n+1)*(n_order+1):n_order+1+(n+1)*(n_order+1)) = -getCoeffCons(0)*getM(n_order);
    end


    %#####################################################
    % 组合所有约束构成等式约束 
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end