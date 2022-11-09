close all; clear all; clc

% parameters
g = 9.81; L = 0.5;

% P controller
K = [0.1 0 25 0.2];

% simulation
tspan = 0:0.01:20;

u=@(y) K*y; % control law
y0 = [0 0 deg2rad(5) 0];
fprintf("Simulating...")

[t,y] = ode45(@(t,y) simpend(y,L,g,u(y)),tspan,y0);
fprintf("done\n")

% plot position and angle vs time
title('P Controller Simulation')
xlabel('Time (s)')
grid on

yyaxis left
plot(t,rad2deg(y(:,3)),'LineWidth',2)
ylabel('$\theta$ (deg)','Interpreter','latex')
hold on

yyaxis right
plot(t,y(:,1),'LineWidth',2)
ylabel('Position (m)')
hold off

%% Save Animation
% create a fun animation, save to a video
fprintf("Creating animation...")
f = figure('Visible','off');
v = VideoWriter('InvertedPendulumP');
v.FrameRate = 10;
open(v);
for k=1:10:length(t)
    drawpend(y(k,:),L);
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);
fprintf("done\n")

%% Plot Input Signal
f = figure('Visible', 'on');
U = u(y);
plot(t,U);
title("u(t)")
xlabel('Time (s)')
ylabel('Force (N)')
legend('u')
grid on