function [goal] =  local_goal(x, path)

index = 0;
closeDist = 100;
for idx = 1 : length(path(:, 1))
    dist =  norm(x(1:2) - path(idx, :)');
    if dist < closeDist
        closeDist = dist;
        index = idx;
    end
end
if index < length(path(:, 1))
    index = index + 1;
end
goal = path(index, :);
