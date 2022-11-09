function dx = simpend(x,M,F,g,L,u)

a = (x(3)-x(1))/L;

% equations of motion for a pendulum on cart (Kwakernaak)
dx(1,1) = x(2);
dx(2,1) = -F/M*x(2) + 1/M*u;
dx(3,1) = x(4);
dx(4,1) = g*sin(a) - F/M*(1-cos(a))*x(2) + 1/M*(1-cos(a))*u;
end


