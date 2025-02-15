function sdot=quadEOM_readonly(t,true_state,F,M,Fd)
    global params

    xdot=true_state(4);
    ydot=true_state(5);
    zdot=true_state(6);
    qW=true_state(7);
    qX=true_state(8);
    qY=true_state(9);
    qZ=true_state(10);
    p=true_state(11);
    q=true_state(12);
    r=true_state(13);
    Rot=QuatToRot([qW,qX,qY,qZ]');
    [phi,theta,yawangle]=RotToRPY_ZXY(Rot);


    sdot=zeros(13,1);

    BRW=[cos(yawangle)*cos(theta)-sin(phi)*sin(yawangle)*sin(theta), cos(theta)*sin(yawangle)+cos(yawangle)*sin(phi)*sin(theta), -cos(phi)*sin(theta);...
         -cos(phi)*sin(yawangle), cos(phi)*cos(yawangle), sin(phi);...
         cos(yawangle)*sin(theta)+cos(theta)*sin(phi)*sin(yawangle), sin(yawangle)*sin(theta)-cos(yawangle)*cos(theta)*sin(phi), cos(phi)*cos(theta)];
    WRB=BRW';


    accel=1/params.mass*(WRB*([0;0;F]+Fd)-[0;0;params.mass*params.grav]);

    K_quat=2;
    % 用于四元数误差修正，确保为单位四元数
    quaterror=1-(qW^2+qX^2+qY^2+qZ^2);
    qdot=-1/2*[0,-p,-q,-r;...
               p,0,-r,q;...
               q,r,0,-p;...
               r,-q,p,0]*[qW,qX,qY,qZ]'+K_quat*quaterror*[qW,qX,qY,qZ]';

    omega=[p;q;r];
    pqrdot=params.invI*(M-cross(omega,params.I*omega));


    sdot(1)=xdot;
    sdot(2)=ydot;
    sdot(3)=zdot;
    sdot(4)=accel(1);
    sdot(5)=accel(2);
    sdot(6)=accel(3);
    sdot(7)=qdot(1);
    sdot(8)=qdot(2);
    sdot(9)=qdot(3);
    sdot(10)=qdot(4);
    sdot(11)=pqrdot(1);
    sdot(12)=pqrdot(2);
    sdot(13)=pqrdot(3);
