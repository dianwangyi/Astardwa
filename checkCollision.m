function [validTraj] = checkCollision(traj, obTraj, R)

trajSave = traj;
validTraj = [];
flag = true;

for Ind = 1 : 5 : length(traj(:, 1))
    for t = 1 : length(traj(1, :))              %% 记录动态障碍物，只与动态障碍物比对  比对之后排除碰撞的轨迹  再在剩余的中找最优的
       for obInx = 1 : 5 : length(obTraj(:, 1))
           dist = norm(traj(Ind : Ind+1, t) - obTraj(obInx : obInx+1, t));  %%有问题 应该和10个障碍物都比较
           if dist <  R
                flag = false;
                break;
           end
       end
       
       if flag == false
           break;
       end
    end
    if flag == true
        validTraj = [validTraj; trajSave(Ind : Ind + 4, : )];
    end
    flag = true;
end