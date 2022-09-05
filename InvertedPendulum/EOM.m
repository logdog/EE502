%% Create the equations of motion in state-space form
clear all; close all; clc
syms x(t) theta(t) M m L g u;

% assumptions
assume(M,'positive');
assume(m,'positive');
assume(L,'positive');
assume(g,'positive');

% derivatives
x_dot = diff(x,t);
x_ddot = diff(x,t,t);

theta_dot = diff(theta,t);
theta_ddot = diff(theta,t,t);

% coordinates of m1 mass
xm = x + L*sin(theta);
ym = L*cos(theta);

xm_dot = diff(xm,t);
ym_dot = diff(ym,t);

% Potential energy
V = m*g*ym;

% Kinetic energy
T = 1/2*M*x_dot^2 + 1/2*m*(xm_dot^2+ym_dot^2);

% Lagrangian
L = T-V;

% EOM (x_ddot)
eqn1 = diff(diff(L,x_dot),t) - diff(L,x) == u;
eqn1 = simplify(eqn1);
eqn1 = isolate(eqn1,x_ddot);

% EOM (theta_ddot)
eqn2 = diff(diff(L,theta_dot),t) - diff(L,theta) == 0;
eqn2 = simplify(eqn2);
eqn2 = isolate(eqn2,theta_ddot);

% substitute (x,x_dot,theta,theta_dot,phi,phi_dot)
syms x1 x2 x3 x4 x2_dot x4_dot
old_vars = [x,x_dot,x_ddot,theta,theta_dot,theta_ddot];
new_vars = [x1,x2,x2_dot,x3,x4,x4_dot];
eqn1 = subs(eqn1,old_vars,new_vars);
eqn2 = subs(eqn2,old_vars,new_vars);

% Solve the EOM in terms of state variables
sln = solve([eqn1,eqn2],[x2_dot,x4_dot])

%% Linearize equations around nominal trajectory
A = sym('A',[4 4]);
B = sym('B',[4 1]);
x0 = [0 0 0 0]'; % nominal trajectory

eqn = [x2; sln.x2_dot; x4; sln.x4_dot];
var = [x1; x2; x3; x4];

% take partial derivatives
for i=1:4
    for j=1:4
        A(i,j) = diff(eqn(i),var(j));
    end
    B(i) = diff(eqn(i),u);
end

% evaluate at the nominal trajectory
A = subs(A,var,x0)
B = subs(B,var,x0)