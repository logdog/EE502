close all; clear all; clc

% parameters
m = 0.01; M = 1; g = 9.81; L = 0.5;

% state space model (obtained from EOM.m)
A =[[0, 1,               0, 0]
    [0, 0,        -(g*m)/M, 0]
    [0, 0,               0, 1]
    [0, 0, (M*g + g*m)/(M*L), 0]];
B = [0; 1/M; 0; -1/(2*M)];

% LQR
Q = diag([1,1,10,10]);
R = 1000;
K = lqr(A,B,Q,R);

% simulation
tspan = 0:0.001:10;
k = 1:length(tspan);
wr = [0; 0; 0; 0]; % reference position
u=@(y)-K*(y - wr); % control law
y0 = [0 0 -deg2rad(5) 0];
fprintf("Simulating...")
[t,y] = ode45(@(t,y)simpend(y,M,m,L,g,u(y)),tspan,y0);
fprintf("done\n")

%% Save Animation
% create a fun animation, save to a video
% fprintf("Creating animation...")
% f = figure('Visible','off');
% v = VideoWriter('InvertedPendulumLQR');
% open(v);
% for k=1:30:length(t)
%     drawpend(y(k,:),L);
%     frame = getframe(gcf);
%     writeVideo(v,frame);
% end
% close(v);
% fprintf("done\n")

%% Plot Angles vs Time
% f = figure('Visible', 'on');
% plot(t,rad2deg(y(:,3)));
% title("Angle vs Time")
% xlabel('Time (s)')
% ylabel('Angle (deg)')
% legend('\theta')
% grid on

%% Plot Input Signal
f = figure('Visible', 'on');
U = u(y(k,:)');
plot(t,U);
title("u(t)")
xlabel('Time (s)')
ylabel('Force (N)')
legend('u')
grid on

%% Plot Velocity
f = figure('Visible', 'on');

uDiff = (U(2:end) - U(1:end-1))/.001;
plot(t(1:end-1),y(1:end-1,2)); hold on;
plot(t(1:end-1),uDiff)

title('$\dot{x}(t)$','Interpreter','latex')
xlabel('Time (s)')
legend('$\dot{x}(t)$','$\dot{u}(t)$','Interpreter','latex')
grid on