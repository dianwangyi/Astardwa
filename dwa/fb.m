%% 障碍物的运动模型
function x = fb(x, u)
global dt;
F = [ 1 0 0 0 0
        0 1 0 0 0
        0 0 1 0 0
        0 0 0 0 0
        0 0 0 0 0];

B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0           dt 
        1           0
        0           1];
    
x= F*x+B*u;
end
