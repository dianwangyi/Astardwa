%% 衡量当前移动机器人与距离它最近的障碍物的距离
function obstaclecost = ObstacleCost(distObst,safeDist)
   delt = safeDist - distObst;
   obstaclecost = max(delt,0);
end