function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    dF = 4*2 + (n_seg-1);
    dP = 3*(n_seg-1);
    Ct = zeros(4*2+4*2*(n_seg-1), dF+dP);
    
    j = 1;
    j_p = dF+1;
    rows = size(Ct,1);
    for i = 1:rows
        if i<=4 || i>=rows-3
            Ct(i,j) = 1;
            j = j+1;
        elseif mod(i,4)==1
            Ct(i,j) = 1;
            if mod(i,8)==1
                j = j+1;
            end
        else
            Ct(i,j_p) = 1;
            j_p = j_p + 1;
            if mod(i,8)==0
                j_p = j_p-3;
            end
        end
    end

end