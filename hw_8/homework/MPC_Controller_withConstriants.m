function [U, u]= MPC_Controller_withConstriants(x,F,H,M,Beta_bar,b,p)
% 利用二次规划求解系统控制（输入）
U = quadprog(H,F*x,M,Beta_bar+b*x,[],[],[],[]);
% 根据模型预测控制的策略，仅选取所得输入的第一项， 参考（5.3.18）
u = U(1:p,1);
end