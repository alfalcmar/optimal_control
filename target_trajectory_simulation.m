clc
clear all
close all

%%%%%%%%%%%%%%%%%%%% set parameters of the target simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set time and waypoint of the target trajectory

t = [0 5 10];
points = [-6 -4; 10 -4; 15 6];

%% set obstacles positions and radius

obspoints = [-2 -2;];
radius_obst = [4];

%% target circle shape

target_radius = 0.5;

%% predicted trajectory

horizon_predicted_trajectory = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% CALL THE SOLVER %%%%%%%%%%%%%%%%

%% Comment this block if you do not have the forces pro license
% Set initial guess to start solver from:

% initializing solver



model.lb = [-0.2 -0.2 -0.2 -10 -10 0   0 0 0];
model.ub = [+0.2 +0.2 +0.2 +10 +10 +30 1 1 1];

model.N = 30;            % horizon length
model.xfinal = [6; 6; 20; 0; 0; 0]; % v final=0 (standstill)


x0i = model.lb+(model.ub-model.lb)/2;
x0=repmat(x0i',model.N,1);
problem.x0=x0; 

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:
problem.xinit = [-10; 5; 5; 0; 0; 0];
problem.xfinal = model.xfinal;


%%%%%%%%%%%%%%%%%%%%%%% CALCULATE TARGET TRAJECTORY%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


x = points(:,1);
y = points(:,2);

tq = 0:0.1:10;
slope0 = 0;
slopeF = 0;

pos_x = spline(t,[slope0; x; slopeF],tq);
pos_y = spline(t,[slope0; y; slopeF],tq);

[m n] = size(obspoints);
figure;

%%%%%%%%%% PLOT OBSTACLES %%%%%%%%%%%%%%%%%%%%%%

for i = 1: length(m);
obs = rectangle('Position',[obspoints(i,1) obspoints(i,2) radius_obst(1) radius_obst(1)],'Curvature',[1,1],'FaceColor','r');
end
grid

%%%%%%%%%%%%%%% MAIN LOOP %%%%%%
%% In this loop the solver si called and the trajectory of the target and the drone are plotted

for k = 1: length(pos_x)-1

%% call the solver
problem.xfinal(1) = pos_x(k);
problem.xfinal(2) = pos_y(k);
problem.xfinal(3) = 6;

[output,exitflag,info] = FORCESNLPsolver(problem);
%%%%%%
xCenter =pos_x(k);
yCenter = pos_y(k); 

d = target_radius*2;
px = xCenter-0.5;
py = yCenter-0.5;
h = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor','b');
h3 = rectangle('Position',[output.x02(4) output.x02(5) d d],'Curvature',[1,1],'FaceColor','g');

hold on
h1 = plot(pos_x(1:k),pos_y(1:k),'--+r');
h2 = plot(pos_x(k+1:k+horizon_predicted_trajectory), pos_y(k+1:k+horizon_predicted_trajectory),'--+g');
for i=1:30
    TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
    x_temp = TEMP(4,:);
    y_temp = TEMP(5,:);
    z_temp = TEMP(6,:);
end

h4 = plot(x_temp',y_temp','--')
axis([points(1,1)-5 points(end,1)+10 points(1,2)-5 points(end,2)+10])


pause(0.1);


set(h,'Visible','off')
set(h2,'Visible', 'off')
set(h3, 'Visible', 'off')
set(h4, 'Visible', 'off')

legend('obstacles','target', 'target trajectory', 'predicted trajectory')

problem.xinit(1) = output.x03(4);
problem.xinit(2) = output.x03(5);
problem.xinit(3) = output.x03(6);
problem.xinit(4) = output.x03(7);
problem.xinit(5) = output.x02(8);
problem.xinit(6) = output.x02(9);

%%% plot trajectory calculated by the solver

for i=1:30
    TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
    x_temp = TEMP(4,:);
    y_temp = TEMP(5,:);
    z_temp = TEMP(6,:);
end


end




