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