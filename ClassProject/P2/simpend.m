function dy = simpend(y,L,g,u)
% u is a 1x1 input scalar
x_dot = y(2);
theta = y(3);
theta_dot = y(4);

dy(1,1) = x_dot;
dy(2,1) = u;
dy(3,1) = theta_dot;
dy(4,1) = g*sin(theta)/L - u*cos(theta)/L;

end

