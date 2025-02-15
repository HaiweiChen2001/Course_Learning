function s_des = circle_trajectory(t, true_s)
    
    s_des = zeros(13,1);

    omega=25;      
    x_des=4*cos(t*omega/180*pi);
    y_des=4*sin(t*omega/180*pi);
    z_des=3/25*t;          

    x_vdes=-omega/180*pi*4*sin(t*omega/180*pi);   
    y_vdes= omega/180*pi*4*cos(t*omega/180*pi);
    z_vdes=3/25;           

    s_des(1)=x_des;
    s_des(2)=y_des;
    s_des(3)=z_des;
    s_des(4)=x_vdes;
    s_des(5)=y_vdes;
    s_des(6)=z_vdes;
    
    %desired yaw angle in the flight
    % 利用 mod 函数确保偏航角始终在 [0, 2 * pi) 范围内
    des_yaw = mod(0.1*pi*t, 2*pi);
    ypr = [des_yaw, 0.0, 0.0];  % 欧拉角，Z-Y-X顺序
    Rot = ypr_to_R(ypr);    % 欧拉角转换为旋转矩阵
    q_des = R_to_quaternion(Rot);   % 旋转矩阵转换为四元数
    s_des(7:10) = q_des;
end
