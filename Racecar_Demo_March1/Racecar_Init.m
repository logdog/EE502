x1_0 = 0;
x1_dot_0 = 0;
x2_0 = 0;
x2_dot_0 = 0;

% LQR controller
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0 0; 1 0; 0 0; 0 1];
Q = diag([1,1,1,1]);
R = diag([1,2]);
K = lqr(A,B,Q,R);