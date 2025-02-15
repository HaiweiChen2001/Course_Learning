function s_des = diamond_trajectory(t, true_s)
    % 三维空间中钻石形状由4条直线段组成
    s_des = zeros(13,1);
    
    t1=t; Period=12; %Diamond Period
    Seg=sqrt(2);    % 轨迹每一直线段的长度 
    Vel=Seg*4/Period;   % 飞行器每一段的速度
    x_des=4/25*t; 
    x_vdes=4/25;
    T1=Period/4; T2=Period/2; T3=3*Period/4;
    
    if(t1>Period)
        t1=t1-Period;
    end

    if(t1<=T1)
        y_vdes=Vel;
        z_vdes=Vel;
        y_des=y_vdes*t1;
        z_des=z_vdes*t1;
    elseif(T1<t1 && t1<=T2)
        y_vdes=Vel;
        z_vdes=-1*Vel;    
        y_des=Seg+y_vdes*(t1-Period/4);
        z_des=Seg+z_vdes*(t1-Period/4);
    elseif(T2<t1 && t1<=T3)
        y_vdes=-1*Vel;
        z_vdes=-1*Vel;
        y_des=2*Seg+y_vdes*(t1-Period/2);
        z_des=z_vdes*(t1-Period/2);
    else
        y_vdes=-1*Vel;
        z_vdes=Vel;
        y_des=Seg+y_vdes*(t1-3*Period/4);
        z_des=-Seg+z_vdes*(t1-3*Period/4);
    end

    s_des(1)=x_des; s_des(2)=y_des; s_des(3)=z_des; s_des(4)=x_vdes; s_des(5)=y_vdes; s_des(6)=z_vdes;
    %desired yaw angle in the flight
    des_yaw = mod(0.2 * pi * t,2 * pi);
    ypr = [des_yaw, 0.0, 0.0];
    Rot = ypr_to_R(ypr);
    q_des = R_to_quaternion(Rot);
    s_des(7:10) = q_des;
end
