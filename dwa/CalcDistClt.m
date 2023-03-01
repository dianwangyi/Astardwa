function [dist_clost_node] = CalcDistClt(xt, path)

if isempty(path)
    dist_clost_node = 0;
    return;
end

dist_clost_node = 100;
clostIdx = 0;
for idx = 1 : length(path(:, 1))
    dist =  norm(xt(1:2) - path(idx, :)',2);
    if dist < dist_clost_node
        dist_clost_node = dist;
        clostIdx = idx;
    end
end


if clostIdx < length(path(:,1))
    clostIdx = clostIdx + 1;
    dist_clost_node = norm(xt(1:2) - path(clostIdx, :)' , 2);
end

dist_clost_node = 3 - dist_clost_node;

% if dist_clost_node < 0.5 
%     dist_clost_node = 0;
% else
   % dist_clost_node = 3 - dist_clost_node;
% end
