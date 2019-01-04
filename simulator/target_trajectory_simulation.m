clc
clear all
close all

%%%%%%%%%%%%%%%%%%%% set parameters of the target simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set time and waypoint of the target trajectory

t = [0 5 10];
points = [4 8; 12 8; 17 20];

%% set obstacles positions and radius

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



model.lb = [-0.5 -0.5 -0.5 -30 -30 0   0 0 0];
model.ub = [+0.5 +0.5 +0.5 +30 +30 +30 1 1 1];

model.N = 30;            % horizon length
%model.xfinal = [6; 6; 20; 0; 0; 0]; % v final=0 (standstill)


x0i = model.lb+(model.ub-model.lb)/2;
x0=repmat(x0i',model.N,1);
problem.x0=x0;

%param = [-5; -10; 20];
%problem.all_parameters=repmat(param, model.N,1);

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:
problem.xinit = [-5; -15; 0; 0; 0; 0];
%%%%%%%%%%%%%%%%%%%%%%% CALCULATE TARGET TRAJECTORY%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


x = points(:,1);
y = points(:,2);

tq = 0:0.1:10;
slope0 = 0;
slopeF = 0;

pos_x = spline(t,[slope0; x; slopeF],tq);
pos_y = spline(t,[slope0; y; slopeF],tq);



figure;
%%%%%%%%%% CALCULE OBSTACLES TRAJECTORY %%%%%%%%%%%%%

t_obs = [0 3 6];
points_obs= [-2 1; 6 -3; 11 -8];

x_obs = points_obs(:,1);
y_obs = points_obs(:,2);

tq_obs = 0:0.1:10;
slope0_obs = 0;
slopeF_obs = 0;

pos_x_obs = spline(t,[slope0_obs; x_obs; slopeF_obs],tq_obs);
pos_y_obs = spline(t,[slope0_obs; y_obs; slopeF_obs],tq_obs);

obspoints = [pos_x_obs; pos_y_obs];
[m n] = size(obspoints);



%%%%%%%%%% PLOT OBSTACLES %%%%%%%%%%%%%%%%%%%%%%

%  for i = 1: 1;
%  obs = rectangle('Position',[obspoints(1,i) obspoints(2,i) radius_obst(1) radius_obst(1)],'Curvature',[1,1],'FaceColor','r');
%  end
% % grid

%%%% uncomment if you want to record the simulation %%%%
%vidObj = VideoWriter('peaks.avi');
%vidObj.FrameRate = 10;
%open(vidObj);
%%%%%%%%%%%%%%% MAIN LOOP %%%%%%
%% In this loop the solver si called and the trajectory of the target and the drone are plotted

for k = 1: length(pos_x)-1

%% call the solver

%param = [pos_x(k); pos_y(k); 20];
%problem.all_parameters=repmat(param, model.N,1);
%problem.xfinal=[pos_x(k);pos_y(k);5; 0; 0; 0];
 param = [pos_x(k); pos_y(k); 6;0 ;0;0;obspoints(1,k)+3;obspoints(2,k)+3];
 problem.all_parameters = repmat(param,30,1);
[output,exitflag,info] = FORCESNLPsolver(problem);
 if exitflag == 1
    xCenter =pos_x(k);
    yCenter = pos_y(k); 

    d = target_radius*2;
    px = xCenter-0.5;
    py = yCenter-0.5;
    obs_plot = rectangle('Position',[obspoints(1,k) obspoints(2,k) radius_obst(1) radius_obst(1)],'Curvature',[1,1],'FaceColor','r');
    h = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor','b');
    h3 = rectangle('Position',[output.x02(4) output.x02(5) d d],'Curvature',[1,1],'FaceColor','g');

    hold on
    h1 = plot(pos_x(1:k),pos_y(1:k),'--+r','MarkerSize',3);
    %h2 = plot(pos_x(k+1:k+horizon_predicted_trajectory), pos_y(k+1:k+horizon_predicted_trajectory),'--+g');
    for i=1:30
        TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
        x_temp = TEMP(4,:);
        y_temp = TEMP(5,:);
        z_temp = TEMP(6,:);
    end

    h4 = plot(x_temp',y_temp','--');
    axis([-10 20 -20 30])


    pause(0.1);
    currFrame = getframe;
    %writeVideo(vidObj,currFrame);

    set(h,'Visible','off')
    %set(h2,'Visible', 'off')
    set(h3, 'Visible', 'off')
    set(h4, 'Visible', 'off')
    set(obs_plot,'visible','off')

    %legend('obstacles','target', 'target trajectory', 'predicted trajectory')

    problem.xinit(1) = output.x02(4);
    problem.xinit(2) = output.x02(5);
    problem.xinit(3) = output.x02(6);
    problem.xinit(4) = output.x02(7);
    problem.xinit(5) = output.x02(8);
    problem.xinit(6) = output.x02(9);

    %%% plot trajectory calculated by the solver

    for i=1:30
        TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
        x_temp = TEMP(4,:);
        y_temp = TEMP(5,:);
        z_temp = TEMP(6,:);
    end

    z(k) = output.x02(6);
 else
    %% sigue con la anterior 
 end
end

height_target = zeros(1,100);
figure(2)
plot(tq(1,1:100),z,'g');
hold on
plot(tq(1,1:100),height_target, 'b')
legend('drone height','target height')
xlabel('time[s]')
ylabel('height[m]')
%close(vidObj);


