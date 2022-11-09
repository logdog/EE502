close all; clear all; clc

% parameters
m = 0.01; M = 1; g = 9.81; L = 0.5;

% P controller
K = [0 0 15 0];

% simulation
tspan = 0:0.01:20;

u=@(y) K*y; % control law
y0 = [0 0 -deg2rad(5) 0];
fprintf("Simulating...")

[t,y] = ode45(@(t,y) simpend(y,M,m,L,g,u(y)),tspan,y0);
fprintf("done\n")

%% Theta
f = figure('Visible', 'on');
plot(t,rad2deg(y(:,3)));
title("Angle vs Time")
xlabel('Time (s)')
ylabel('Angle (deg)')
legend('\theta')
grid on

%% Position
f = figure('Visible', 'on');
title("x(t)")
plot(t,y(:,1))
xlabel('Time (s)')
ylabel('position m)')
legend('x')
grid on

%% Save Animation
% create a fun animation, save to a video
fprintf("Creating animation...")
f = figure('Visible','off');
v = VideoWriter('InvertedPendulumP');
open(v);
for k=1:3:length(t)
    drawpend(y(k,:),L);
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);
fprintf("done\n")

%% Plot Input Signal
f = figure('Visible', 'on');
k = 1:length(tspan);
U = u(y(k,:)');
plot(t,U);
title("u(t)")
xlabel('Time (s)')
ylabel('Force (N)')
legend('u')
grid on