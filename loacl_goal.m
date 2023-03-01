function [goal] =  local_goal(x, path)

index = 0;
closeDist = 100;
for idx = 1 : length(path(:, 1))
    dist =  norm(x(1:2) - path(idx, :)',2);
    if dist < closeDist
        closeDist = dist;
        index = idx;
    end
end
closestNode = path(index, :);
