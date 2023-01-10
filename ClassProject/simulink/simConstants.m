% accleration due to gravity
g = 9.81;

% Effective length of pendulum arm
L = 0.35;

% Time delay in angle sensors
Td = 100e-6;

% Sample Time (how fast is arduino running?)
Ts = 1e-3;

% initial condition
theta0 = deg2rad(5);

% lqr control
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 g/L 0];
B = [0; 1; 0; -1/L];
Q = diag([1 1 10 10]);
R = 1;
F = lqr(A,B,Q,R)
Fd = lqrd(A,B,Q,R,1e-3)

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

% solve for Kalman Filter using MATLAB tools (got the same answer!)
% sensors = [1];
% known = [];
% [~,L,~] = kalman(sys2,0,V2,0,sensors,known)
% 
% solve for Discrete-Time Kalman Filter
sys2 = ss(AA,BB,CC,0);
sys2d = c2d(sys2,Ts)
sensors = [1];
known = [];
[~,Ld,~] = kalman(sys2d,0,V2,0,sensors,known,'current')

% A-KC
A_KC_d = sys2d.A - Ld*sys2d.C
B_d = sys2d.B