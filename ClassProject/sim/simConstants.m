% accleration due to gravity
g = 9.81;

% Effective length of pendulum arm
L = 0.2;

% Time delay in angle sensors
Td = 50e-6;

% Sample Time (how fast is arduino running?)
Ts = 50e-6;

% initial condition
theta0 = deg2rad(5);

% lqr control
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 g/L 0];
B = [0; 1; 0; -1/L];
Q = diag([1 1 10 0]);
R = 100;
K = lqr(A,B,Q,R,0);