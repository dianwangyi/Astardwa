% astar dwa 主程序
% astar产生参考路径点
% 在dwa中引入参考点
% -----------------------------------------------------------------------------
%参考：https://blog.csdn.net/heyijia0327/article/details/44983551
%2022/6/8 : 构建障碍物的运动学模型，障碍物改成动态,速度跟车辆速度一致
%          加入小车动画

%%

close all;
clear all;

disp('Dynamic Window Approach sample program start!!')

%% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
% x=[0 0 pi/2 0 0]'; % 5x1矩阵 列矩阵  位置 0，0 航向 pi/2 ,速度、角速度均为0
x = [0.1 0.1 pi/10 0 0]';

% 下标宏定义 状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
POSE_X      = 1;  %坐标 X
POSE_Y      = 2;  %坐标 Y
YAW_ANGLE   = 3;  %机器人航向角
V_SPD       = 4;  %机器人速度
W_ANGLE_SPD = 5;  %机器人角速度

goal = [10,10];   % 目标点位置 [x(m),y(m)]

% xb = [3 2 0]';
xb = [6 7 0 0 0;
        4 4 0 0 0;
        2 5 0 0 0;
        1 2 0 0 0; % dy
        4 4 pi 0 0;
        7 3 pi/2 0 0;
        7 7 0 0 0; % dy
        6 4 0 0 0;
        8 9 0 0 0;
        9 6 0 0 0]';
    
    ub = [0 0;
            0 0;
            0 0;
            0 0;
            0.3 0;
            0.5 0;
            0.5 0;
            0 0;
            0 0;
            0 0]';

% obstacle = [6 6;
%     4 4;
%     2 5;
%     1 2; % Dynamic
%     4 4;
%     7 3;
%     7 9; % Dynamic
%     6 4;
%     8 9;
%     9 6];
obstacleR = 0.5;% 冲突判定用的障碍物半径


map.XYMAX = 10;
map.start = [0, 0];
map.goal = goal;

%path = AStar(xb(1:2, :)', map);
[path, ~, ~]=AStar_24(xb(1:2, :)', map);%24_A*算法
path = flip(path);

path = KeyNode(path);

globalgoal = goal;
localgoal = path(2, :);

global dt;
dt = 0.1;% 时间[s]

% 机器人运动学模型参数
% 最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]]
Kinematic = [2.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];
%定义Kinematic的下标含义
MD_MAX_V    = 1;%   最高速度m/s]
MD_MAX_W    = 2;%   最高旋转速度[rad/s]
MD_ACC      = 3;%   加速度[m/ss]
MD_VW       = 4;%   旋转加速度[rad/ss]
MD_V_RESOLUTION  = 5;%  速度分辨率[m/s]
MD_W_RESOLUTION  = 6;%  转速分辨率[rad/s]]


% 评价函数参数 [heading,dist,velocity,predictDT]
% 1.航向得分的比重、2.距离得分的比重、3.速度得分的比重、
% 4. deter_v, 5.deter_dist, 6. dist_node, 7. dist_goal, 8.向前模拟轨迹的时间
evalParam = [0.2, 0.2 ,0.3, 0.0, 0.3, 0.0, 0, 3.0];

area      = [-1 11 -1 11];% 模拟区域范围 [xmin xmax ymin ymax]

% 模拟实验的结果
result.x=[];   %累积存储走过的轨迹点的状态值
tic; % 估算程序运行时间开始
%%   小车动画
figure('color',[1 1 1])
h1 = axes();
data = my_gritsbot_patch;
this.robot_body = data.vertices;
th = x(YAW_ANGLE)-pi/2;
rotation_matrix = [
    cos(th) -sin(th) x(POSE_X);
    sin(th)  cos(th) x(POSE_Y);
    0 0 1];
transformed = this.robot_body*rotation_matrix';
% movcount=0;
%% Main loop   循环运行 5000次 指导达到目的地 或者 5000次运行结束
edge_flag = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0 ];
max_flag = 0; % 障碍物到达最大边界的标志
min_flag = 0; % 障碍物到达最小边界的标志

%% draw picture
for i = 1:5000
    
    tmpgoal = local_goal(x, path);
    if tmpgoal ~= localgoal
        localgoal
    end
    
    localgoal = tmpgoal;
    
    obstacle = [];
    for inxOb = 1 : length(xb(1, :))
%         if xb(1, inxOb) >= 8
%             max_flag = 1;
%             min_flag = 0;
%         elseif xb(1, inxOb) <= 0
%             max_flag = 0;
%             min_flag = 1;
%         end
%         if max_flag == 1
%             ub = -ub;  % 障碍物的速度，
%         elseif min_flag == 1
%             ub = ub;
%         end
        if (xb(1, inxOb) >= 8) || (xb(2, inxOb) >= 8) || (xb(1, inxOb) <= 0) || (xb(2, inxOb) <= 0)   
            edge_flag(inxOb) = 1;
        else 
            edge_flag(inxOb) = 0;
        end
        if edge_flag(inxOb) == 1
            ub(1, inxOb) = -ub(1, inxOb);
        end
        xb(:, inxOb) = fb(xb(:, inxOb),ub(:, inxOb));
        xbi = xb(1:2, inxOb);
        xbi = xbi';
        obstacle = [obstacle; xbi];
    end
    
    
    % DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹
    [u,traj] = DynamicWindowApproach(x,Kinematic,localgoal,evalParam,obstacle,obstacleR, path, xb, ub);
    x = f(x,u);% 机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度
    

        
%     ub = [0.7,0]';
%     if xb(1) >= 8
%         max_flag = 1;
%         min_flag = 0;
%     elseif xb(1) <= 0
%         max_flag = 0;
%         min_flag = 1;
%     end
%     if max_flag == 1
%         ub = -ub;  % 障碍物的速度，
%     elseif min_flag == 1
%         ub = ub;
%     end
%     xb = fb(xb,ub);
%     obstacle = [0 2;
%         2 4;
%         2 5;
%         10-xb(1) 2;
%         4 10-xb(1);
%         6 xb(1);
%         5 9
%         8 5
%         xb(1) 8
%         9 9];
%     obstacle(4,1) = 10-xb(1);
%     obstacle(5,2) = 10-0.7*xb(1);
%     obstacle(6,2) = 0.5*xb(1);
%     obstacle(7,1) = 0.5*xb(1);
    
    % 历史轨迹的保存
    result.x = [result.x; x'];  %最新结果 以列的形式 添加到result.x
    
    % 是否到达目的地
    if norm(x(POSE_X:POSE_Y)-goal')<0.5   % norm函数来求得坐标上的两个点之间的距离
        if localgoal == globalgoal 
            disp('Arrive Goal!!');break;
        end
    end
    
    delete(h1)  % 新图出现时，取消原图的显示。防止重复绘图。
    h1 = axes();
    ArrowLength = 0.5;      % 箭头长度
    hold on
    th = x(YAW_ANGLE)-pi/2;
    rotation_matrix = [
        cos(th) -sin(th) x(POSE_X);
        sin(th)  cos(th) x(POSE_Y);
        0 0 1];
    transformed = this.robot_body*rotation_matrix';
  
    patch(...
    'Vertices', transformed(:, 1:2), ...
    'Faces', data.faces, ...
    'FaceColor', 'flat', ...
    'FaceVertexCData', data.colors, ...
    'EdgeColor','none');  % 绘制小车
    
    hold on;
    plot(result.x(:,POSE_X),result.x(:,POSE_Y),'-b');hold on;    % 绘制走过的所有位置 所有历史数据的 X、Y坐标
    plot(goal(1),goal(2),'*r');hold on;                          % 绘制目标位置
    
    plot(path(:,1),path(:,2),'sr','MarkerFaceColor','r');hold on
    plot(path(:,1), path(:, 2), '--r');hold on;
    
    DrawObstacle_plot(obstacle,obstacleR);          % 绘制障碍物
    
 
      
    % 探索轨迹 画出待评价的轨迹
    if ~isempty(traj) %轨迹非空
        for it=1:length(traj(:,1))/5    %计算所有轨迹数  traj 每5行数据 表示一条轨迹点
            ind = 1+(it-1)*5; %第 it 条轨迹对应在traj中的下标
            plot(traj(ind,:),traj(ind+1,:),'--g');hold on  %根据一条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的所有x坐标值  traj(ind+1,:)表示第ind条轨迹的所有y坐标值
        end
    end
    
    axis(area); %根据area设置当前图形的坐标范围，分别为x轴的最小、最大值，y轴的最小最大值
    xlabel('$x(m)$','interpreter','latex','FontSize',12);
    ylabel('$y(m)$','interpreter','latex','FontSize',12);
    grid on;
    %% 保存为gif
        frame=getframe(gcf);
         im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i==1
             imwrite(imind,cm,'dynamic.gif','gif', 'LoopCount',inf,'DelayTime',0.000001);
        end
        if rem(i,2)==0
             imwrite(imind,cm,'dynamic.gif','gif','WriteMode','append','DelayTime',0.000001);
        end
    
    drawnow;  %刷新屏幕. 当代码执行时间长，需要反复执行plot时，Matlab程序不会马上把图像画到figure上，这时，要想实时看到图像的每一步变化情况，需要使用这个语句。
  
end
toc  %输出程序运行时间  形式：时间已过 ** 秒。



