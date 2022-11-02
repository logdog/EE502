function drawpend(state,L)

% state(3) = s + L theta, => theta = (state(3) - s)/L

x = state(1);
th = (state(3) - x)/L;

% dimensions
W = 0.5;  % cart width
H = 0.25; % cart height
wr = 0.1;         % wheel radius
mr = 0.1;  % mass radius

% positions
y = wr/2+H/2; % cart vertical position

px1 = x + L*sin(th);
py1 = y + L*cos(th);

plot([-10 10],[0 0],'k','LineWidth',2), hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
rectangle('Position',[x-.9*W/2,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
rectangle('Position',[x+.9*W/2-wr,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel

plot([x px1],[y py1],'k','LineWidth',2); % Draw pendulum
rectangle('Position',[px1-mr/2,py1-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);

axis([-5 5 -(L+0.5) L+0.5]); 
axis equal
set(gcf,'Position',[100 100 1000 400])
grid on
drawnow, hold off