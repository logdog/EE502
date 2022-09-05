clear all;
syms theta(t) m g L

% assumptions on parameters
assume(m,'positive')
assume(L,'positive')
assume(g,'positive')

% other state variables
theta_dot = diff(theta,t);
theta_ddot = diff(theta_dot,t);

% coordinates
xm = L*sin(theta);
ym = L-L*cos(theta);

% derivatives
xm_dot = diff(xm,t);
ym_dot = diff(ym,t);

% Potential Energy
V = m*g*ym;

% Kinetic Energy
T = 1/2*m*(xm_dot^2 + ym_dot^2);

% Lagrangian
L = T - V;

% equations of motion
eqn1 = diff( diff(L,theta_dot) ,t) - diff(L,theta) == 0;

% solve for theta_ddot
eqn1 = simplify(eqn1);
isolate(eqn1,theta_ddot)