%% Double Pendulum on Cart - Uncontrolled
close all; clear all; clc

% parameters
M=10; m1=1; m2=2;
L1=2; L2=1.7; g=9.81;

% run the simulation
tspan = 0:0.001:7;
y0 = [0 0 deg2rad(-1) 0 deg2rad(1) 0]; % initial state

fprintf("Simulating...")
[t,y] = ode45(@(t,y)simpend(y,M,m1,m2,L1,L2,g,0),tspan,y0);
fprintf("done\n")

%% Save Animation
% create a fun animation, save to a video
fprintf("Creating animation...")
f = figure('Visible','off');
v = VideoWriter('double_pend_cart_free');
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
close all