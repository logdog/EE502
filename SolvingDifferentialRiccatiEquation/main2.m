clear; clc;
%% input system parameters
A = eye(2);
B = eye(2);
Q = eye(2);
R = eye(2); Ri = inv(R);

% boundary condition Pf
Pf = eye(2);
Pf = Pf(:);

% simulation start, end
tf = 1000;
to = 0;

% numerically back-solve for P (three different boundary conditions)
[T,P1] = ode45(@(t,P)mRiccati(P,Q,Ri,A,B), [tf to], [0;0;0;0]);
[T,P2] = ode45(@(t,P)mRiccati(P,Q,Ri,A,B), [tf to], 10*Pf);
[T,P3] = ode45(@(t,P)mRiccati(P,Q,Ri,A,B), [tf to], 100*Pf);

% plot the evolution of P. Notice how P(0) is the same for all cases
figure
plot(P1(:,1)); hold on; grid on;
plot(P2(:,1));
plot(P3(:,1));
legend('P1','P2','P3')
xlabel('Time (reversed)')