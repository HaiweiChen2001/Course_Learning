clc; clear; close all;

%% 参数
Np=20;
dt=0.2;
A = [0 1 0; 0 0 1; 0 0 0];
B = [0; 0; 1];
B = [B, zeros(3,1)];
n = size(A,1);
p = size(B,2);
C = eye(n);
D = zeros(n,p);

S = eye(n);
S(1) = 10;
Q = eye(n);
Q(1) = 10;
R = eye(p);
R(end) = 1e4;

% 离散化
sys = c2d(ss(A,B,C,D), dt);
A = sys.A;
B = sys.B;

%% 离散系统转换为标准二次型
Phi = zeros(n*Np,n);
for i = 1:Np
    Phi((i-1)*n+1:i*n, :) = A^i;
end

Gamma = zeros(n*Np, p*Np);
Gamma(1:n, 1:p) = B;
for i = 1:Np-1
    Gamma(i*n+1:(i+1)*n, 1:i*p) = A*Gamma((i-1)*n+1:i*n, 1:i*p);
    Gamma(i*n+1:(i+1)*n, i*p+1:(i+1)*p) = B;
end


%% 性能指标转化为标准二次型
Omega = kron(eye(Np-1), Q);
Omega = blkdiag(Omega, S);

Psi = kron(eye(Np), R);

F = Gamma'*Omega*Phi;
H = Gamma'*Omega*Gamma+Psi;


%% 约束转化为标准二次型
x_low = [-inf; -1; -1];
x_high = [inf; 1; 1];
u_low = [-inf; 0];
u_high = [inf; inf];

[M,Beta_bar,b] = Soft_MPC_Matrices_Constraints(x_low,x_high,u_low,u_high,Np,Phi,Gamma);

%% 求解
x0 = [10; -3; 0];
x = x0;
log = [0 x(1) x(2) x(3)];

for t=0.2:0.2:20
    J = quadprog(H, F*x, M, Beta_bar+b*x);
    j = J(1:p, 1);
    x = A*x + B*j;
    log = [log; t x(1) x(2) x(3)];
end

plot(log(:,1), log(:,2));
hold on;
plot(log(:,1), log(:,3));
plot(log(:,1), log(:,4));
grid on;

