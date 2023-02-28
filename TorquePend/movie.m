% grab simulation data
sim_t = out.theta_deg.time;
[sim_theta, usat, mode] = out.theta_deg.signals.values;

% interpolate
x_1 = @(t) L*sind(interp1(sim_t, sim_theta, t, 'spline'));
y_1 = @(t) -L*cosd(interp1(sim_t, sim_theta, t, 'spline'));

% animate
ax1 = subplot(3,3,[1 2 4 5 7 8]);

args = {"FrameRate", 30, "AnimationRange", [0 18]};
fanimator(ax1, @(t) plot(x_1(t),y_1(t),'bo','MarkerSize',10,'MarkerFaceColor','b'), args{:});
xlim([-1.1*L 1.1*L]);
grid on;
axis equal square;
hold on;
fanimator(ax1, @(t) plot([0 x_1(t)],[0 y_1(t)],'b-'), args{:});
fanimator(ax1, @(t) text(-0.3,0.3+L,"Time: "+num2str(t,3)), args{:});
hold off;

ax2 = subplot(3,3,3);
fanimator(ax2, @(t) plot(0:1/30:t, interp1(sim_t, sim_theta, 0:1/30:t, 'spline')), args{:});
hold on;
fanimator(ax2, @(t) plot(t, interp1(sim_t, sim_theta, t, 'spline'), 'bo'), args{:});
ylabel('$\theta$ (deg)','Interpreter','latex')
hold off;

ax3 = subplot(3,3,6);
fanimator(ax3, @(t) plot(0:1/30:t, interp1(sim_t, usat, 0:1/30:t, 'linear'), 'r'), args{:});
hold on;
fanimator(ax3, @(t) plot(t, interp1(sim_t, usat, t, 'linear'), 'ro'), args{:});
ylabel('u_{sat}')
hold off;

ax4 = subplot(3,3,9);
fanimator(ax4, @(t) plot(0:1/30:t, interp1(sim_t, mode, 0:1/30:t, 'nearest'), 'g'), args{:});
hold on;
fanimator(ax4, @(t) plot(t, interp1(sim_t, mode, t, 'nearest'), 'go'), args{:});
ylabel('mode')
xlabel('Time (s)')
hold off;

writeAnimation("swingup_balance.gif",'LoopCount',Inf);