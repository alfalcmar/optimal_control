clc
clear all
close all

%%%%%%%%%%%%%%%%%%%% set parameters of the target simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set time and waypoint of the target trajectory

t = [0 2 4 6 8 10];
points = [0 0; 1 2; 1 3; 2 4; 5 8; 7 10];

%% set obstacles positions and radius

obspoints = [4 4;];
radius_obst = [2];

%% target circle shape

target_radius = 0.5;

%% predicted trajectory

horizon_predicted_trajectory = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = points(:,1);
y = points(:,2);

tq = 0:0.1:10;
slope0 = 0;
slopeF = 0;

pos_x = spline(t,[slope0; x; slopeF],tq);
pos_y = spline(t,[slope0; y; slopeF],tq);

[m n] = size(obspoints);
figure;
for i = 1: length(m);
obs = rectangle('Position',[obspoints(i,1) obspoints(i,2) radius_obst(1) radius_obst(1)],'Curvature',[1,1],'FaceColor','r');
end

for k = 1: length(pos_x)
 
 
xCenter =pos_x(k);
yCenter = pos_y(k); 

d = target_radius*2;
px = xCenter-0.5;
py = yCenter-0.5;
h = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor','b');
hold on
h1 = plot(pos_x(1:k),pos_y(1:k),'--+r');
h2 = plot(pos_x(k+1:k+horizon_predicted_trajectory), pos_y(k+1:k+horizon_predicted_trajectory),'--+g');
axis([points(1,1)-1 points(end,1)+1 points(1,2)-1 points(end,2)+1])
pause(0.1);
set(h,'Visible','off')
set(h2,'Visible', 'off')

legend('obstacles','target', 'target trajectory', 'predicted trajectory')
end

grid



