% accleration due to gravity
g = 9.81;

% Effective length of pendulum arm
L = 0.35;

% Time delay in angle sensors
Td = 100e-6;

% Sample Time (how fast is arduino running?)
Ts = 50e-6;

% initial condition
theta0 = deg2rad(5);

% lqr control
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 g/L 0];
B = [0; 1; 0; -1/L];
Q = diag([1 1 10 10]);
R = 1;
F = lqr(A,B,Q,R);

%P = icare(A,B,Q,R)

% root locus
C = [0 0 -1 0];
sys = ss(A,B,C,0);

% sysC = tf([1 5],[1 20]);
% sisotool(sysC*sys)

% Kalman filtering
AA = [0 1; g/L 0];
BB = [0; -1/L];
CC = [1 0];
V2 = 20e-6;

W = icare(AA',CC',0,V2);
K = W * CC' / V2