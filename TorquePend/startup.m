% pendulum constants
m = 1;
L = 1;
g = 9.81;
b = 0;

% initial conditions
theta0 = 0;
theta_dot0 = 0;

% swingup controller gain
k=1;

% LQR controller
A = [0 1; g/L 0];
B = [0; 1/(m*L^2)];
Q = 10*eye(2);
R = 1;
[K,S] = lqr(A,B,Q,R);
