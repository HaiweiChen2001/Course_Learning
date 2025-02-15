function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;


qW = s(7);
qX = s(8);
qY = s(9);
qZ = s(10);
Rot = QuatToRot([qW,qX,qY,qZ]');
[phi, theta, yaw]=RotToRPY_ZXY(Rot);

qW = s_des(7);
qX = s_des(8);
qY = s_des(9);
qZ = s_des(10);
Rot = QuatToRot([qW,qX,qY,qZ]');
[~, ~, yaw_c] = RotToRPY_ZXY(Rot);


%% 位置环
P_kp = [10; 10; 10];
P_kd = [10; 10; 10];

for i = 1:3
    acc(i) = 0 + P_kd(i)*(s_des(4+i-1)-s(4+i-1)) + P_kp(i)*(s_des(i)-s(i));
end
ss
F = m*(g+acc(3));

phi_c = 1/g*(acc(1)*sin(yaw) - acc(2)*cos(yaw));
theta_c = 1/g*(acc(1)*cos(yaw) + acc(2)*sin(yaw));


%% 姿态环
angle_kp = [900; 900; 900];
angle_kd = [50; 50; 50];

dyaw = yaw_c-yaw;
if dyaw<-pi
    dyaw = dyaw + 2*pi;
elseif dyaw>pi
    dyaw = dyaw - 2*pi;
end

angle_acc = zeros(3,1);
angle_acc(1) = angle_kp(1)*(phi_c-phi) + angle_kd(1)*(s_des(11)-s(11));
angle_acc(2) = angle_kp(2)*(theta_c-theta) + angle_kd(2)*(s_des(12)-s(12));
angle_acc(3) = angle_kp(3)*(dyaw) + angle_kd(3)*(s_des(13)-s(13));

M = I*angle_acc + cross(s(11:13), params.I*s(11:13));

%F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M
end
