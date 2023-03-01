function [dist_goal] = CalaGoal(xt, goal)

dist_goal = 20 - norm(xt(1:2) - goal(:)',2);
