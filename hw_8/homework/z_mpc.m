function [log_z] = z_mpc(A, B, C, D, dt, Np, S, Q, R)
    % Z 轴
    u= 0;
    z = [20; 0; 0];
    zd = [20; -0.5; 0];
    za = [z; zd; 0];
    log_z = [z' zd'];

    p = size(B,2);

    Ad = c2d(ss([0 1 0; 0 0 1; 0 0 0],B,C,D), dt).A;
    % 输入增量控制
    [Aa, Ba, Ca, Sa, Qa, R] = InputAugmentMatrix_Delta(A, B, Ad, S, Q, R);
    % 性能指标转换
    [Phi, Gamma, Omega, Psi, H, F]= MPC_Matrices_PM(Aa, Ba, Qa, R, Sa, Np);

    u_low = -2;
    u_high = 2;
    x_low = [-inf; -1; -1];
    x_high = [inf; 6; 3];
    xa_low = [x_low; -inf; -inf; -inf; u_low];
    xa_high = [x_high; inf; inf; inf; u_high];
    [M, Beta_bar, b] = MPC_Matrices_Constraints(xa_low, xa_high, u_low, u_high, Np, Phi, Gamma);

    for k = 0.2:0.2:40        
        [delta_U, delta_u]= MPC_Controller_withConstriants(za,F,H,M,Beta_bar,b,p);
        u = delta_u+u;
        z = A*z + B*u;  
        % zd = [20-0.5*k; -0.5; 0];
        zd = Ad*zd;
        za = [z; zd; u];
        log_z = [log_z; z' zd'];
    end
end