function dy = simpend(y,g,L)
%SIMPEND Equations of motion for pendulum
%   y = state vector y(1)=theta, y(2)=theta_dot
%   L = length of mass-less rod
%   g = acceleration due to gravity (>0)
dy(1,1) = y(2);
dy(2,1) = -g*sin(y(1))/L^2;
end

