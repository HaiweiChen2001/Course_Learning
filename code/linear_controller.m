function [F, M] = controller(t, s, s_des)

global params

global rms_xposition;
global rms_xvelocity;
global rms_yposition;
global rms_yvelocity;
global rms_zposition;
global rms_zvelocity;
% persistent t_last;
% 
% if isempty(t_last)
%     t_last = 0.0;
% end
% 
% t_last = t;
% deltT = t - t_last;


m = params.mass;
g = params.grav;
I = params.I;

%Current State
z=s(3); 
x=s(1);
y=s(2);
dz=s(6); dx=s(4); dy=s(5);
dphi=s(11);
dtheta=s(12);
dvarphi=s(13);
Quad0=s(7:10);

[phi,theta,varphi]=RotToRPY_ZXY(quaternion_to_R(Quad0));

R_ab=[cos(theta),0,-cos(phi)*sin(theta);0,1,sin(phi);sin(theta),0,cos(phi)*cos(theta)];
W_xyz=R_ab*[dphi;dtheta;dvarphi];
w_x=W_xyz(1);
w_y=W_xyz(2);
w_z=W_xyz(3);

% Desire State
z_des=s_des(3); x_des=s_des(1); y_des=s_des(2);
dz_des=s_des(6); dx_des=s_des(4); dy_des=s_des(5);   
QuadC=s_des(7:10);

[phiC,thetaC,varphiC] = RotToRPY_ZXY(quaternion_to_R(QuadC));
ddz_des=0; ddx_des = 0; ddy_des = 0;
dphiC=0;dthetaC=0;dvarphiC=0;

%PID Parameter
Kd_z=15;
Kp_z=30;
Kd_x=10;
Kp_x=10;
Kd_y=10;
Kp_y=10;
%
Kp_phi=900;  Kd_phi=50;
Kp_theta=900; Kd_theta=50;
Kp_varphi=900; Kd_varphi=50;

%Position Control
ddz = ddz_des+Kd_z*(dz_des-dz)+Kp_z*(z_des-z); % PID z
ddx = ddx_des+Kd_z*(dx_des-dx)+Kp_x*(x_des-x); % PID x
ddy = ddy_des+Kd_z*(dy_des-dy)+Kp_y*(y_des-y); % PID y
u_1 = m*(g+ddz); % Z-axis thrust

phiC = 1/g*(ddx*sin(varphi)-ddy*cos(varphi));
thetaC = 1/g*(ddx*cos(varphi)+ddy*sin(varphi));


%yaw
if(((varphiC-varphi)>-pi)&&((varphiC-varphi)<pi))
    dYaw=varphiC-varphi;
elseif((varphiC-varphi)<=-pi)
    dYaw=varphiC-varphi + 2 * pi;
elseif((varphiC-varphi)>=pi)
    dYaw=varphiC-varphi - 2 * pi;
end

%pitch
if(((thetaC-theta)>-pi)&&((thetaC-theta)<pi))
    dTheta=thetaC-theta;
elseif((thetaC-theta)<=-pi)
    dTheta=thetaC-theta + 2 * pi;
elseif((thetaC-theta)>=pi)
    dTheta=thetaC-theta - 2 * pi;
end

%roll
if(((phiC-phi)>-pi)&&((phiC-phi)<pi))
    dPhi=phiC-phi;
elseif((phiC-phi)<=-pi)
    dPhi=phiC-phi + 2 * pi;
elseif((phiC-phi)>=pi)
    dPhi=phiC-phi - 2 * pi;
end



%Attitude Control

ddphiC = Kp_phi*(phiC-phi)+Kd_phi*(dphiC-dphi);
ddthetaC= Kp_theta*(thetaC-theta)+Kd_theta*(dthetaC-dtheta);
ddvarphiC= Kp_varphi*(dYaw)+Kd_varphi*(dvarphiC-dvarphi); %PID
u_2 = I*[ddphiC;ddthetaC;ddvarphiC]+cross([w_x;w_y;w_z],I*[w_x;w_y;w_z]);
F = u_1;
M = u_2;

rms_xposition=[rms_xposition,s_des(1)-s(1)];
rms_xvelocity=[rms_xvelocity,s_des(4)-s(4)];
rms_yposition=[rms_yposition,s_des(2)-s(2)];
rms_yvelocity=[rms_yvelocity,s_des(5)-s(5)];
rms_zposition=[rms_zposition,s_des(3)-s(3)];
rms_zvelocity=[rms_zvelocity,s_des(6)-s(6)];

% F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M
% M = [0.0, 0.0, 0.0]';
end
