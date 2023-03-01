%% DWA算法实现
% model  机器人运动学模型  最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss], 速度分辨率[m/s],转速分辨率[rad/s]]
% 输入参数：当前状态、模型参数、目标点、评价函数的参数、障碍物位置、障碍物半径
% 返回参数：控制量 u = [v(m/s),w(rad/s)] 和 轨迹集合 N * 31  （N：可用的轨迹数）
% 选取最优参数的物理意义：在局部导航过程中，使得机器人避开障碍物，朝着目标以较快的速度行驶。
function [u,trajDB] = DynamicWindowApproach(x,model,goal,evalParam,ob,R)
% Dynamic Window [vmin,vmax,wmin,wmax] 最小速度 最大速度 最小角速度 最大角速度速度
Vr = CalcDynamicWindow(x,model);  % 根据当前状态 和 运动模型 计算当前的参数允许范围

% 评价函数的计算 evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
%               trajDB      每5行一条轨迹 每条轨迹都有状态x点串组成
[eval,evalDB,trajDB]= Evaluation(x,Vr,goal,ob,R,model,evalParam);  %evalParam 评价函数参数 [heading,dist,velocity,predictDT]


% dete_dist = CalcDistEval(x,ob(4:7,:),R)
dete_dist = min(evalDB(:,7));
if dete_dist < 1
      u=[0;0];return;
end

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

% 各评价函数正则化
evalDB = NormalizeEval(evalDB);

% 最终评价函数的计算
feval=[];
for id=1:length(evalDB(:,1))
    feval = [feval;evalParam(1:5)*evalDB(id,3:7)']; %根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得分
end
evalDB = [evalDB feval]; % 最后一组

[maxv,ind] = max(feval);% 选取评分最高的参数 对应分数返回给 maxv  对应下标返回给 ind

% forwardGain = 10;
% obstacleGain = 100;
% feval = [];
% if sum(eval(:,1))~= 0
%     eval(:,1) = eval(:,1)/sum(eval(:,1));  %矩阵的数除  单列矩阵的每元素分别除以本列所有数据的和
% end
% if sum(eval(:,2))~= 0
%     eval(:,2) = eval(:,2)/sum(eval(:,2));
% end
% for id = 1:length(eval(:,1))
%     feval =[feval;forwardGain*eval(id,1)-obstacleGain*eval(id,2)];
% end
% [maxv,ind] = max(feval);

u = evalDB(ind,1:2)';% 返回最优参数的速度、角速度
end