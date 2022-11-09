function drawpend(state,L)

x = state(1);
th = state(3);

% dimensions

px1 = x + L*sin(th);
py1 = L*cos(th);


plot([x 1],[0 0],'k','LineWidth',1); hold on
axis([-1 1 -1 1]);
plot([x px1],[0 py1],'k','LineWidth',2);
grid on
drawnow, hold off