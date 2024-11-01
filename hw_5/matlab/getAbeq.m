function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    % #####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    T = 0;
    for k = 0:3 % p,v,a,j
        for i = k:n_order
            Aeq_start(k+1, i+1) = factorial(i)/factorial(i-k)*(T^(i-k));
        end
    end
    beq_start = start_cond';

    %#####################################################
    % p,v,a,j constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    T = ts(end);
    idx = (n_seg-1)*(n_order+1);
    for k = 0:3
        for i = k:n_order
            Aeq_end(k+1, idx+i+1) = factorial(i)/factorial(i-k)*(T^(i-k));
        end
    end
    beq_end = end_cond';
   
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for n=0:n_seg-1-1
        T = ts(n+1);
        idx = n*(n_order+1);
        for i=0:n_order
            Aeq_wp(n+1,idx+i+1) = T^i;
        end
    end
    beq_wp = waypoints(2:end-1);
    
    % ####################################################
    % Continuity Constrain
    Aeq_con = zeros((n_seg-1)*4, n_all_poly);
    beq_con = zeros((n_seg-1)*4, 1);
    for n = 0:n_seg-1-1
        Aeq_con(1+4*n:4+4*n, 1+n*(n_order+1):n_order+1+n*(n_order+1)) = getCoeffCons(ts(n+1));
        Aeq_con(1+4*n:4+4*n, 1+(n+1)*(n_order+1):n_order+1+(n+1)*(n_order+1)) = -getCoeffCons(0);
    end

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end