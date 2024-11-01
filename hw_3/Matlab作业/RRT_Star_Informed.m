%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化
clear all; close all; clc;
x_I=1; y_I=1;           % 设置初始点
%x_G=770; y_G=450;       % 设置目标点
x_G=500; y_G=500;       % 设置目标点
Thr=50;                 %设置目标点阈值
Delta=30;               % 设置扩展步长

iter_count = 999;
path_find=false;
best_cost = 1000000;
%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     %
%% 开始构建树——作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%地图x轴长度
yL=size(Imp,2);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count=1;
for iter = 1:iter_count
    x_rand=[];
    %Step 1: 在地图中随机采样一个点x_rand
    %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标
    if path_find
        x_rand = SampleInEllipse([x_I,y_I], [x_G, y_G], best_cost, [0,xL,0,yL]);
    else
        x_rand = rand(1,2) .* [xL, yL];
    end

    x_near=[];
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    %提示：x_near已经在树T里
    min_dist = 1000000;
    near_iter = 1;
    [~, N] = size(T.v);
    for j = 1:N
        x_near(1) = T.v(j).x;
        x_near(2) = T.v(j).y;
        dist = norm(x_near-x_rand);
        if min_dist > dist
            min_dist = dist;
            near_iter = j;
        end
    end
    x_near(1) = T.v(near_iter).x;
    x_near(2) = T.v(near_iter).y;

    x_new=[];
    %Step 3: 扩展得到x_new节点
    %提示：注意使用扩展步长Delta
    dis = norm(x_rand-x_near);
    x_new = fix(((dis-Delta)*x_near + Delta*x_rand)/dis);

    %检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
        continue;
    end

    % nearC && chooseParent
    nearPtr = [];
    nearCount = 0;
    nearDist = norm(x_new-x_near) + T.v(near_iter).dist;
    for j = 1:N
        if j == near_iter
            continue;
        end
        x_neartmp(1) = T.v(j).x;
        x_neartmp(2) = T.v(j).y;
        norm_dist = norm(x_new-x_neartmp);
        if norm_dist < 120
            %nearC
            if collisionChecking(x_neartmp, x_new, Imp)
                nearCount= nearCount + 1;
                nearPtr(nearCount, 1) = j;
                dist = norm(x_new-x_neartmp)+ T.v(j).dist;
                if nearDist > dist
                    nearDist = dist;
                    near_iter = j;
                end
            end
        end
    end

    x_near(1) = T.v(near_iter).x;
    x_near(2) = T.v(near_iter).y;

    count=count+1;
    %Step 4: 将x_new插入树T 
    %提示：新节点x_new的父节点是x_near
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_near(1);
    T.v(count).yPrev = x_near(2);
    T.v(count).dist = norm(x_new-x_near) + T.v(near_iter).dist; 
    T.v(count).indPrev = near_iter;

    % rewire
    [M, ~] = size(nearPtr);
    for k = 1:M
        x_l(1) = T.v(nearPtr(k,1)).x;
        x_l(2) = T.v(nearPtr(k,1)).y;
        xl_prev(1) = T.v(nearPtr(k,1)).xPrev;
        xl_prev(2) = T.v(nearPtr(k,1)).yPrev;
        if T.v(nearPtr(k,1)).dist > (T.v(count).dist + norm(x_l-x_new))
            T.v(nearPtr(k,1)).dist = T.v(count).dist + norm(x_l-x_new);
            T.v(nearPtr(k,1)).xPrev = x_new(1);
            T.v(nearPtr(k,1)).yPrev = x_new(2);
            T.v(nearPtr(k,1)).indPrev = count;
            plot([x_l(1),xl_prev(1)], [x_l(2),xl_prev(2)], '-w');
            hold on;
            plot([x_l(1),x_new(1)], [x_l(2),x_new(2)], '-g');
            hold on;
        end
    end

    %Step 6:将x_near和x_new之间的路径画出来
    %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
    %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来
    plot([x_near(1),x_new(1)], [x_near(2),x_new(2)], '-r');
    hold on;

    %Step 5:检查是否到达目标点附近 
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    goal = [x_G, y_G];
    if norm(goal-x_new) < Thr
        new_cost = T.v(end).dist + norm(x_new-goal);
        if best_cost > new_cost
            path_find = true;
            best_cost = new_cost;
            best_id = count;
            disp(['Path found with cost：', num2str(best_cost)]);
        end
    end
   
   pause(0.01); %暂停0.1s，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if path_find
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(best_id).x; path.pos(2).y = T.v(best_id).y;
    pathIndex = T.v(best_id).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


