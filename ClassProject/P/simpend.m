function dy = simpend(y,M,m,L,g,u)
% u is a 1x1 input scalar

x3 = y(3); % theta
x4 = y(4); % theta_dot

dy(1,1) = y(2);
dy(2,1) = (L*m*sin(x3)*x4^2 + u - g*m*cos(x3)*sin(x3))/(- m*cos(x3)^2 + M + m);
dy(3,1) = y(4);
dy(4,1) = -(L*m*cos(x3)*sin(x3)*x4^2 + u*cos(x3) - g*m*sin(x3) - M*g*sin(x3))/(L*(- m*cos(x3)^2 + M + m));

end

