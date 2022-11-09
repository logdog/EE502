%% Create the equations of motion in state-space form
syms x(t) theta(t) phi(t) M m1 m2 L1 L2 g u;

% assumptions
assume(M,'positive');
assume(m1,'positive');
assume(m2,'positive');
assume(L1,'positive');
assume(L2,'positive');
assume(g,'positive');

% derivatives
x_dot = diff(x,t);
x_ddot = diff(x,t,t);

theta_dot = diff(theta,t);
theta_ddot = diff(theta,t,t);

phi_dot = diff(phi,t);
phi_ddot = diff(phi,t,t);

% coordinates of m1 mass
xm1 = x + L1*sin(theta);
ym1 = L1*cos(theta);

xm1_dot = diff(xm1,t);
ym1_dot = diff(ym1,t);

% coordinates of m2 mass
xm2 = xm1 + L2*sin(phi);
ym2 = ym1 + L2*cos(phi);

xm2_dot = diff(xm2,t);
ym2_dot = diff(ym2,t);

% Potential energy
V = m1*g*ym1 + m2*g*ym2;

% Kinetic energy
T = 1/2*M*x_dot^2 + 1/2*m1*(xm1_dot^2+ym1_dot^2) + 1/2*m2*(xm2_dot^2+ym2_dot^2);

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

% EOM (phi_ddot)
eqn3 = diff(diff(L,phi_dot),t) - diff(L,phi) == 0;
eqn3 = simplify(eqn3);
eqn3 = isolate(eqn3,phi_ddot);

% substitute (x,x_dot,theta,theta_dot,phi,phi_dot)
syms x1 x2 x3 x4 x5 x6 x2_dot x4_dot x6_dot
old_vars = [x,x_dot,x_ddot,theta,theta_dot,theta_ddot,phi,phi_dot,phi_ddot];
new_vars = [x1,x2,x2_dot,x3,x4,x4_dot,x5,x6,x6_dot];
eqn1 = subs(eqn1,old_vars,new_vars);
eqn2 = subs(eqn2,old_vars,new_vars);
eqn3 = subs(eqn3,old_vars,new_vars);

% Solve the EOM in terms of state variables
sln = solve([eqn1,eqn2,eqn3],[x2_dot,x4_dot,x6_dot])

%% Linearize equations around nominal trajectory
A = sym('A',[6 6]);
B = sym('B',[6 1]);
x0 = [0 0 0 0 0 0]'; % nominal trajectory

eqn = [x2; sln.x2_dot; x4; sln.x4_dot; x6; sln.x6_dot];
var = [x1; x2; x3; x4; x5; x6];

% take partial derivatives
for i=1:6
    for j=1:6
        A(i,j) = diff(eqn(i),var(j));
    end
    B(i) = diff(eqn(i),u);
end

% evaluate at the nominal trajectory
A = subs(A,var,x0)
B = subs(B,var,x0)