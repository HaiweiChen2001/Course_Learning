function [M,Beta_bar,b] = Soft_MPC_Matrices_Constraints(x_low,x_high,u_low,u_high,Np,Phi,Gamma)
    n = size(x_low,1);
    p = size(u_low,1);

    M = [zeros(p,n);zeros(p,n);-eye(n);eye(n)];
    F = [-eye(p);eye(p);zeros(n,p);zeros(n,p)];
    F(2*p+2, 2) = -1;
    Beta = [-u_low;u_high;-x_low;x_high];

    M_Np = [-eye(n); eye(n)];
    Beta_Np = [-x_low;x_high];

    M_bar = zeros((2*n+2*p)*Np+2*n,n);
    M_bar(1:(2*n+2*p),:) = M;
    Beta_bar = [repmat(Beta,Np,1);Beta_Np];
    
    % 初始化M_2bar矩阵
    M_2bar = M;
    % 初始化F_2bar矩阵
    F_2bar = F;
    
    % for循环创建M_2bar和F_2bar矩阵
    for i=1:Np-2
       M_2bar = blkdiag(M_2bar, M);
       F_2bar = blkdiag(F_2bar, F);
    end
    M_2bar = blkdiag(M_2bar,M_Np);
    M_2bar = [zeros(2*n+2*p,n*Np);M_2bar];
    F_2bar = blkdiag(F_2bar,F);
    F_2bar = [F_2bar; zeros(2*n,p*Np)];

    M = M_2bar*Gamma+F_2bar;
    b = -(M_bar+M_2bar*Phi);
end