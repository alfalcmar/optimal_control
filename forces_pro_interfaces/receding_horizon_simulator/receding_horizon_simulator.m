clc, clear all, close all

% change for your local path
addpath('/home/grvc/Desktop/casadi')
addpath('/home/grvc/Desktop/FORCES_PRO_CLIENT')
import casadi.*;

%% parameters
cntrl_freq = 10; % control frequency (Hz)
simulation_time = 10;  % duration of the simulation (seg)
radius = 4; %no fly zone dimension (m)
final_pose = [7.65 -55 3]; %final pose
final_vel = [0 0 0];
t_init = [-8.5 -29.5]; %initial target pose
tvx_init = (final_pose(1)-t_init(1))/sqrt((final_pose(1)-t_init(1))^2+(final_pose(2)-t_init(2))^2); % initial target velocity (cte)
tvy_init = (final_pose(2)-t_init(2))/sqrt((final_pose(1)-t_init(1))^2+(final_pose(2)-t_init(2))^2);
exit = [];
%initial pose
initial_pose = [-18.8 -12.26 3];
%initial vel
v_x_init = 0;
v_y_init = 0;
v_z_init = 0;
% obstacle pose
obst_x = -8.5;
obst_y = -29.5;
N=50; % time horizon

all_trajectories = [];
trajectory = [];

aux2=[];

t_x_temp_aux = [];
t_y_temp_aux = [];

initial_guess = [0.5 0.5 0.5 -18.8 -12.26 3  1 1 2 t_init(1) t_init(2)];

for k = 1: simulation_time
x0 = [];
%% call the solver
% initial guess is the trajectory previously calculated
if k==1
    x0i = initial_guess;
    x0=repmat(x0i',N,1);
else 

   for i=1:length(TEMP)
       x0=[x0;TEMP(:,k)];
   end
end

problem.x0=x0;
param = [final_pose(1); final_pose(2); final_pose(3); final_vel(1); final_vel(2); final_vel(3); obst_x; obst_y; t_init(1); t_init(2); tvx_init ; tvy_init];
problem.all_parameters=repmat(param, N,1);
problem.xinit = [initial_pose(1); initial_pose(2); initial_pose(3); v_x_init; v_y_init; v_z_init; t_init(1); t_init(2)];

% Time to solve the NLP!

[output,exitflag,info] = FORCESNLPsolver(problem);
exit = [exit exitflag]; % saving the output state of the solver
cont = 0;
% if the solver call is not succesfull, we use the last optimal call, and
% we try again at each time step
while(exitflag ~= 1)
    
    cont = cont+1;
    d = radius*1;
    px = obst_x-0.5;
    py = obst_y-0.5;
    pxt = t_init(1)+tvx_init*(k-1)-0.5;
    pyt = t_init(2)+tvy_init*(k-1)-0.5;
    h = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor','b'); %plot obstacle
    h2 = rectangle('Position',[pxt pyt 0.5 0.5],'Curvature',[1,1],'FaceColor','k'); %plot target
    h3 = rectangle('Position',[x_temp(cntrl_freq+cont) y_temp(cntrl_freq+cont) 0.5 0.5],'Curvature',[1,1],'FaceColor','g'); % plot vehicle
    trajectory = [trajectory; x_temp(cntrl_freq+cont) y_temp(cntrl_freq+cont) z_temp(cntrl_freq+cont) v_x_temp(cntrl_freq+cont) v_y_temp(cntrl_freq+cont) v_z_temp(cntrl_freq+cont)];
    axis([-20 10 -55 -10])
    hold on
    h4 = plot(x_temp',y_temp','--'); %plot trajectory calculated by the solver
    pause(1/cntrl_freq);
    currFrame = getframe;
    pause(1/cntrl_freq);

    set(h,'Visible','off')
    set(h2,'Visible','off')
    set(h3, 'Visible', 'off')
    set(h4, 'Visible', 'off')
    
    %% saving the last state to call the solver
    initial_pose(1) = x_temp(cntrl_freq+cont);
    initial_pose(2) = y_temp(cntrl_freq+cont);
    initial_pose(3) = z_temp(cntrl_freq+cont);
    v_x_init = v_x_temp(cntrl_freq+cont);
    v_y_init = v_y_temp(cntrl_freq+cont);
    v_z_init = v_z_temp(cntrl_freq+cont);


    problem.x0=x0;
    param = [final_pose(1); final_pose(2); final_pose(3); final_vel(1); final_vel(2); final_vel(3); obst_x; obst_y; t_init(1)+tvx_init*(k-1); t_init(2)+tvy_init*(k-1); tvx_init ; tvy_init];
    problem.all_parameters=repmat(param, N,1);
    problem.xinit = [initial_pose(1); initial_pose(2); initial_pose(3); v_x_init; v_y_init; v_z_init; t_init(1); t_init(2)];

    % Time to solve the NLP!

    [output,exitflag,info] = FORCESNLPsolver(problem);

    
end

%% plot results
 for i=1:N
     if(N>=100)
        if (i<100)
          TEMP(:,i) = output.(['x0',sprintf('%02d',i)]);
        else
          TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
        end
    else
      TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
     end
 end
        ux_temp = TEMP(1,:);
        uy_temp = TEMP(2,:);
        uz_temp = TEMP(3,:);
        x_temp = TEMP(4,:);
        y_temp = TEMP(5,:);
        z_temp = TEMP(6,:);
        v_x_temp = TEMP(7,:);
        v_y_temp = TEMP(8,:);
        v_z_temp = TEMP(9,:);
        t_x_temp = TEMP(10,:);
        t_x_temp_aux = [t_x_temp_aux TEMP(10,:)];
        t_y_temp = TEMP(11,:);
        t_y_temp_aux = [t_y_temp_aux TEMP(11,:)];

 all_trajectories = [all_trajectories; TEMP(:,1) TEMP(:,2)];
%% controlling the vehicule during 10 steps
dist = norm([x_temp(end)-final_pose(1),y_temp(end)-final_pose(2), z_temp(end)-final_pose(3)])
if dist>1 
    for j=1:cntrl_freq
        d = radius*1;
        px = obst_x-0.5;
        py = obst_y-0.5;
        pxt = t_x_temp(j)-0.5;
        pyt = t_y_temp(j)-0.5;
        h = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor','b'); %plot obstacle
        h2 = rectangle('Position',[pxt pyt 0.5 0.5],'Curvature',[1,1],'FaceColor','k'); %plot target
        h3 = rectangle('Position',[x_temp(j) y_temp(j) 0.5 0.5],'Curvature',[1,1],'FaceColor','g'); % plot vehicle
        if j~=1
            trajectory = [trajectory; ux_temp(j) uy_temp(j) uz_temp(j) x_temp(j) y_temp(j) z_temp(j) v_x_temp(j) v_y_temp(j) v_z_temp(j) t_x_temp(j) t_y_temp(j)];
        end
        axis([-20 10 -55 -10])
        hold on
        h4 = plot(x_temp',y_temp','--'); %plot trajectory calculated by the solver
        pause(1/cntrl_freq);
        currFrame = getframe;
        set(h,'Visible','off')
        set(h2,'Visible','off')
        set(h3, 'Visible', 'off')
        set(h4, 'Visible', 'off')
    end
else % if the final pose is too close, we use the last solver call for the rest of the simulation
    for j=1:N
        d = radius*1;
        px = obst_x-0.5;
        py = obst_y-0.5;
        pxt = t_x_init(j)-0.5;
        pyt = t_y_init(j)-0.5;
        h = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor','b'); %plot obstacle
        h2 = rectangle('Position',[pxt pyt 0.5 0.5],'Curvature',[1,1],'FaceColor','k'); %plot target
        h3 = rectangle('Position',[x_temp(j) y_temp(j) 0.5 0.5],'Curvature',[1,1],'FaceColor','g'); % plot vehicle
        if j~=1
            trajectory = [trajectory; ux_temp(j) uy_temp(j) uz_temp(j) x_temp(j) y_temp(j) z_temp(j) v_x_temp(j) v_y_temp(j) v_z_temp(j) t_x_temp(j) t_y_temp(j)];
        end
        axis([-20 10 -55 -10])
        hold on
        h4 = plot(x_temp',y_temp','--'); %plot trajectory calculated by the solver
        pause(1/cntrl_freq);
        currFrame = getframe;
        set(h,'Visible','off')
        set(h2,'Visible','off')
        set(h3, 'Visible', 'off')
        set(h4, 'Visible', 'off')
    end
 break;
end
    

%% saving the last state to call the solver
initial_pose(1) = x_temp(cntrl_freq);
initial_pose(2) = y_temp(cntrl_freq);
initial_pose(3) = z_temp(cntrl_freq);
v_x_init = v_x_temp(cntrl_freq);
v_y_init = v_y_temp(cntrl_freq);
v_z_init = v_z_temp(cntrl_freq);
t_init(1) = t_x_temp(cntrl_freq);
t_init(2) = t_y_temp(cntrl_freq);

% plot trajectory calculated by the solver

for i=1:N
if(N>=100)
        if (i<100)
          TEMP(:,i) = output.(['x0',sprintf('%02d',i)]);
        else
          TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
        end
    else
      TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
end
    x_temp = TEMP(4,:);
    y_temp = TEMP(5,:);
    z_temp = TEMP(6,:);
end

end

hold on
plot(trajectory(:,1),trajectory(:,2))


