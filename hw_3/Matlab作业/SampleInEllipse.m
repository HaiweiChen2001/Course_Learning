function x_rand = SampleInEllipse(start, goal, path_cost, search_space)
    dist = norm(goal-start);
    center = (start+goal)/2;
    a = path_cost/2;
    c = dist/2;
    b = sqrt(a^2-c^2);

    % 生成随机点在单位圆内
    theta = 2*pi*rand();
    r = rand();
    x_ball = [r*cos(theta), r*sin(theta)];

    % 线性变换映射到椭圆
    x_rand = [a,b] .* x_ball;

    % 旋转并平移到实际空间
    phi = atan2(goal(2)-start(2), goal(1)-start(1));
    rotation = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    x_rand = rotation * x_rand' + center';
    x_rand = x_rand';

    x_rand = min(max(x_rand, [search_space(1), search_space(3)]), [search_space(2), search_space(4)]);

end