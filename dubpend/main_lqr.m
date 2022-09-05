%% Double Pendulum on Cart - LQR
close all; clear all; clc

% parameters
M=10; m1=1; m2=2;
L1=2; L2=1.7; g=9.81;

% state space model (obtained from EOM.m)
A =[[0, 1,                                              0, 0,                           0, 0];
    [0, 0,                     -(g*m1^2 + g*m2*m1)/(M*m1), 0,                           0, 0];
    [0, 0,                                              0, 1,                           0, 0];
    [0, 0, (g*m1^2 + M*g*m1 + M*g*m2 + g*m1*m2)/(L1*M*m1), 0,             -(g*m2)/(L1*m1), 0];
    [0, 0,                                              0, 0,                           0, 1];
    [0, 0,                   -(M*g*m1 + M*g*m2)/(L2*M*m1), 0, (M*g*m1 + M*g*m2)/(L2*M*m1), 0]];

B = [0; 1/M; 0; -1/(L1*M); 0; 0];

% LQR
Q = diag([1,1,100,100,100,100]);
R = 0.001;
K = lqr(A,B,Q,R);

% run the simulation
tspan = 0:0.001:7;
wr = [0; 0; 0; 0; 0; 0]; % reference position
u=@(y)-K*(y - wr); % control law

y0 = [3 0.5 deg2rad(-10) 0 deg2rad(15) 0]; % initial state

fprintf("Simulating...")
[t,y] = ode45(@(t,y)simpend(y,M,m1,m2,L1,L2,g,u(y)),tspan,y0);
fprintf("done\n")

%% Save Animation
% create a fun animation, save to a video
fprintf("Creating animation...")
f = figure('Visible','off');
v = VideoWriter('double_pend_cart_LQR');
open(v);
for k=1:30:length(t)
    drawpend(y(k,:),L1,L2);
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);
fprintf("done\n")

%% Plot Angles vs Time
f = figure('Visible', 'on');
plot(t,rad2deg(y(:,3))); hold on
plot(t,rad2deg(y(:,5)));
title("Angles vs Time")
xlabel('Time (s)')
ylabel('Angle (deg)')
legend('\theta','\phi')
grid on