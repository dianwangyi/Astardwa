%% 绘制所有障碍物位置
% 输入参数：obstacle 所有障碍物的坐标   obstacleR 障碍物的半径
function  DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR;
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
    x = r * cos(theta) + obstacle(id,1);
    y = r  *sin(theta) + obstacle(id,2);
    if id == 4 || id == 5 || id == 6 || id == 7
        plot(x,y,'-k');hold on;
    else
         plot(x,y,'-m');hold on;
    end
   
   
end
plot(obstacle(:,1),obstacle(:,2),'*r');hold on;              % 绘制所有障碍物位置
end