%% Motion Model 根据当前状态推算下一个控制周期（dt）的状态
% u = [vt; wt];当前时刻的速度、角速度 x = 状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
function outx = f(x, u)
global dt;
%     if u(2) == 0    % 直线运动情况
        F = [1 0 0 0 0
        0 1 0 0 0
        0 0 1 0 0
        0 0 0 0 0
        0 0 0 0 0];

    B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0 dt
        1 0
        0 1];
     outx= F*x+B*u;
%      u(2)
%     else
%         r  = (u(1)/u(2));  % 转向半径
%         outx(1,1) = x(1) - r*sin(x(3)) + r*sin(x(3)+u(2)*dt);
%         outx(2,1) = x(2) + r*cos(x(3)) - r*cos(x(3)+u(2)*dt);
%         outx(3,1) = x(3) + dt*u(2);
%         outx(4,1) = u(1);
%         outx(5,1) = u(2);
%      end

end