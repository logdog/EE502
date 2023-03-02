% initial conditions
x0 = 0;
xd0 = 0;

% LQR controllers
A = [0 1; 0 0];
B = [0; 1];
C = eye(2);
D = zeros(2,1);
Q1 = diag([1,1]);
Q2 = diag([1,1]);
R1 = 1;
R2 = 2;

% LQR controller for car 1
K1 = lqr(A,B,Q1,R1);
K2 = lqr(A,B,Q2,R2);