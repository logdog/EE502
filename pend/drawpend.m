function drawpend(theta,L)
%DRAWPEND draws a pendulum
%   theta - angle in radius
%   L - length of rod (>0)
xm = L*sin(theta);
ym = -L*cos(theta);
mr = 1;

% draw rod, draw mass
plot([0 xm],[0 ym],'k','LineWidth',2);
rectangle('Position',[xm-mr/2,ym-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);

axis([-1.5*L 1.5*L -1.5*L 1.5*L]); 
axis square
set(gcf,'Position',[100 100 1000 400])
drawnow, hold off
