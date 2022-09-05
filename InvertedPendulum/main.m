close all; clear all; clc

% parameters
m = 1; M = 3; g = 9.81; L = 2;

% state space model (obtained from EOM.m)
A =[[0, 1,               0, 0]
    [0, 0,            -g/M, 0]
    [0, 0,               0, 1]
    [0, 0, (g + M*g)/(2*M), 0]];
B = [0; 1/M; 0; -1/(2*M)];

% LQR
Q = diag([1,1,100,100]);
R = 0.001;
K = lqr(A,B,Q,R);

% simulation
tspan = 0:0.001:10;
wr = [-3; 0; 0; 0]; % reference position
u=@(y)-K*(y - wr); % control law
y0 = [3 0 -deg2rad(5) 0];
fprintf("Simulating...")
[t,y] = ode45(@(t,y)simpend(y,M,m,L,g,u(y)),tspan,y0);
fprintf("done\n")

%% Save Animation
% create a fun animation, save to a video
fprintf("Creating animation...")
f = figure('Visible','off');
v = VideoWriter('InvertedPendulumLQR');
open(v);
for k=1:30:length(t)
    drawpend(y(k,:),L);
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);
fprintf("done\n")

%% Plot Angles vs Time
f = figure('Visible', 'on');
plot(t,rad2deg(y(:,3)));
title("Angle vs Time")
xlabel('Time (s)')
ylabel('Angle (deg)')
legend('\theta')
grid on
