%{
    DWA main
%}

%{
DWA(robotPose, robotGoal, robotModel)
    laserscan = readScanner();
    allowable_v = generateWindow(robotV, robotModel)
    allowable_w = generateWindow(robotW, robotModel)

    for each v in allowable_v
        for each w in allowable_w

        dist = find_dist(v,w,laserscan,robotModel)
        breakDist = calculateBreakingDistance(v);
        if (dist > breakDist)
            clearance = (dist-breakDist)/(dmax - breakingDist)
            cost = costFunction(heading, clearance, abs(desired_v - v))
            if(cost > optimal)
                best_v = v
                best_w = w
                optimal = cost
    set robot trajection to best_v, best_w
END
%}

function [] = DWASample()

close all;
clear all;

disp('Dynamic Window Approach sample program start!!')

% 5*1列矩阵，机器人的初始状态[x(m) y(m) yaw(rad) v(m/s) w(rad/s)]
x = [0 0 pi/10 0 0]';

POSE_X            = 1; % X坐标
POSE_Y            = 2; % Y坐标
YAW_ANGLE    = 3; % 机器人航向角
V_SPD              = 4; % 机器人速度
W_ANGLE_SPD = 5; % 机器人角速度

goal = [10,10];

obstacle = [0 2; 2 4; 2 5; 4 2; 5 4; 5 5; 5 6; 5 9; 8 8; 8 9;7 9];

obstacleR = 0.5;
global dt;
dt = 0.1;

% 机器人运动学模型
% Kinematic = [最大速度， 最大旋转速度， 加速度， 旋转加速度， 速度分辨率，转速分辨率]
Kinematic = [1.0, toRadian(20.0), 0.2, toRadian(50), 0.01, toRadian(1)];

% 定义Kinematic的下标含义
MD_MAX_V               = 1; % 最大速度
MD_MAX_W              = 2; % 最大角速度
MD_ACC                    = 3; % 加速度
MD_VW                     = 4; % 角加速度
MD_V_RESOLUTION  = 5; % 速度分辨率
MD_W_RESOLUTION = 6; % 角速度分辨率


% 评价函数参数 [heading, dist, velocity, predictDT]
% 航向得分比重、距离得分比重、速度得分比重、向前模拟轨迹的时间
evalParam = [0.05, 0.2, 0.1, 3.0];

% 模拟的区域范围
% [xmin, xmax, ymin, ymax]
area = [-1 11 -1 11];

% 模拟实验的结果
result.x = []; % 累计存储走过的轨迹点的状态值
tic; % 估算程序运行时间开始

%%main loop
for i = 1 : 5000
    % 返回控制量u = [v(m/s), w(rad/s)] 与 轨迹
    [u, traj] = DWA(x, Kinematic, goal, evalParam, obstacle, obstacleR);
    x = f(x,u); % 机器人移动到下一个时刻的状态量， 根据当前速度与角速度推导下一时刻的位置和角度
    
    result.x = [result.x; x]';
    
    if norm(x(POSE_X:POSE_Y) - goal') < 0.5
        disp('Arrive Goal!');
        break;
    end
    
    hold off;
    ArrowLength = 0.5;
    
    quiver(x(POSE_X), x(POSE_Y), ArrowLength * cos(x(YAW_ANGLE)), ArrowLength * sin(x(YAW_ANGLE)));
    hold on;
    
    plot(result.x(:, POSE_X), result.x(:, POSE_Y), '-b'); hold on;
    plot(goal(1), goal(2), '*r'); hold on;
    
    DrawObstacle_plot(obstacle, obstacleR);
    
    
    if ~isempty(traj)
        for it = 1 : length(traj(:, 1) / 5)
            ind = 1 + (it-1)*5;
            plot(traj(ind, :), traj( ind + 1, :), '-g'); hold on;
        end
    end
    
    axis(area);
    grid on;
    
    drawnow;
end
toc


    
    
    
    
    





