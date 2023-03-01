%% 衡量当前移动机器人距离目的地距离与上个时刻距离目的地距离的增量
% xt 当前时刻系统的状态，x上一时刻系统的状态
function goalcost = GoalCost(xt,x,goal)

   Goaldist_xt =  norm(xt(1:2)-goal');
   Goaldist_x =  norm(x(1:2)-goal');
   goalcost = Goaldist_xt - Goaldist_x;
end