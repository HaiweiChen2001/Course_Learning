clc; clear; close all;

%% 参数
w = 0.7;
Np=20;
dt=0.2;
A = [0 1 0; 0 0 1; 0 0 0];
B = [0; 0; 1];
n = size(A,1);
p = size(B,2);
C = eye(n);
D = zeros(n,p);

S = eye(n);
S(1) = 100;
Q = eye(n);
Q(1) = 100;
R = eye(p);

% 初始化

% X 轴
x = [0; 0; 0];
xd = [0; w; 0];
u = 0;
xa = [x; xd; u];

% Y 轴
y = [8; 0; 0];
yd = [0; 0; -w^2];
ya = [y; yd; 0];

% 约束定义
u_low = -3;
u_high = 3;
x_low = [-inf; -6; -3];
x_high = [inf; 6; 3];
xa_low = [x_low; -inf; -inf; -inf; u_low];
xa_high = [x_high; inf; inf; inf; u_high];

% 记录轨迹
log_x = [x' xd'];
log_y = [y' yd'];
%% 离散化
sys = c2d(ss(A,B,C,D), dt);
A = sys.A;
B = sys.B;
Ad = c2d(ss([0 1 0; 0 0 1; 0 -w^2 0],B,C,D), dt).A;

%% 转换
% 输入增量控制
[Aa, Ba, Ca, Sa, Qa, R] = InputAugmentMatrix_Delta(A, B, Ad, S, Q, R);
% 性能指标转换
[Phi, Gamma, Omega, Psi, H, F]= MPC_Matrices_PM(Aa, Ba, Qa, R, Sa, Np);
% 约束条件转换
[M, Beta_bar, b] = MPC_Matrices_Constraints(xa_low, xa_high, u_low, u_high, Np, Phi, Gamma);

%% 计算
% x轴
for k = 0.2:0.2:40
    [delta_U, delta_u]= MPC_Controller_withConstriants(xa,F,H,M,Beta_bar,b,p);
    u = delta_u+u;
    x = A*x + B*u;  
    r = 0.25*k;
    xd = [r*sin(w*k); r*w*cos(w*k); -r*w^2*sin(w*k)];
    xd = Ad*xd;
    xa = [x; xd; u];
    log_x = [log_x; x' xd'];
end

% y轴
u = 0;
for k = 0.2:0.2:40    
    [delta_U, delta_u]= MPC_Controller_withConstriants(ya,F,H,M,Beta_bar,b,p);
    u = delta_u+u;
    y = A*y + B*u;  
    r = 0.25*k;
    yd = [r*cos(w*k); -r*w*sin(w*k); -r*w^2*cos(w*k)];
    yd = Ad*yd;
    ya = [y; yd; u];
    log_y = [log_y; y' yd'];
end

% z轴
log_z = z_mpc(A, B, C, D, dt, Np, S, Q, R);

%% 显示
subplot(3, 1, 1);
plot(1:length(log_x), log_x(:,4), Color='r');
hold on;
plot(1:length(log_x), log_x(:,1), Color='g');

subplot(3, 1, 2);
plot(1:length(log_y), log_y(:,4), Color='r');
hold on;
plot(1:length(log_y), log_y(:,1), Color='g');

subplot(3, 1, 3);
plot(1:length(log_z), log_z(:,4), Color='r');
hold on;
plot(1:length(log_z), log_z(:,1), Color='g');

figure;
plot3(log_x(:,4), log_y(:,4), log_z(:,4), Color='r');
hold on;
plot3(log_x(:,1), log_y(:,1), log_z(:,1), Color='g');
axis equal;
grid on;