clear; clc; close all;
% change for your local path
addpath('/home/grvc/Desktop/casadi')
addpath('/home/grvc/Desktop/FORCES_PRO_CLIENT')
import casadi.*;

%% obstacle and target position
radius = 4;
tx = -8.4; 
ty = -29.5;
final_pose_x =7.65;
final_pose_y =-55;
final_pose_z = 3;

initial_x = -18.8; %-18.8 -12.26
initial_y = -12.26;
initial_z = 3

obst_x = -8.4;
obst_y = -29.5;

%% Problem dimensions
model.N = 50;            % horizon length
model.nvar = 9;          % number of variables
model.neq  = 6;          % number of equality constraints
model.nh = 1;            % number of inequality constraints
model.npar = 12;         % [pfx pfy pfz vxf vyf vzf cx cy tx ty tz vtx vtz]
                         % [1    2   3   4   5   6  7  8  9  10 11 12   13]
       
t= 0.1;  %% time step - integrator

% time step and horizon lenght to use the solver in the receding horizon in
% real experiments


%% z = [ax ay az px py pz vx vy vz]  => [control states]
%% Objective function 

model.objective = @objfun;  %% function objective (included in the same folder)
model.objectiveN = @objfunN; %% N function obective (included in the same folder)

%% MODEL %%%%%%%%%%%%%%%
%% Continous model
% We use an explicit RK4 integrator here to discretize continuous dynamics:
m=1; I=1; % physical constants of the model
integrator_stepsize = 0.1;
continuous_dynamics = @(x,u) [x(4);  % v_x
                              x(5);  % v_y
                              x(6);  % v_z
                              u(1);
                              u(2);
                              u(3)];         

model.eq = @(z) RK4(z(4:9), z(1:3), continuous_dynamics, integrator_stepsize);

%% Discrete model (we are using the discrete model)
%   x_{k+1} = A*x_{k} + B*u_{k}
% A = [I_3 delta_t*I_3; zeros_3 I_3] B = (delta_t^2)/2*I_3; delta_t*I_3]

% A = [eye(3) t*eye(3); zeros(3) eye(3)];
% B = [(t^2)/2*eye(3); t*eye(3)];
% 
% model.eq = @(z) [ A*[z(4);z(5);z(6);z(7);z(8);z(9)] + B*[z(1);z(2);z(3)]];
% 
model.E = [zeros(6,3) eye(6)]; % A(6x6) B(6*3)

%% position, velocities and accelerations limits
% upper/lower variable bounds lb <= x <= ub
model.lb = [-5 -5 -7 -200 -200 0   -12 -12 -1];
model.ub = [+5 +5 7 +200 +200 +50 12 12 3];

%% nonlinear inequalities
% (vehicle_x - obstacle_x)^2 +(vehicle_y - obstacle_y)^2 > r^2
model.ineq = @(z,p)  (z(4)-p(7))^2 + (z(5)-p(8))^2   %% obstacle position as parameter

% Upper/lower bounds for inequalities
model.hu = [inf]';
model.hl = [radius^2]';  %hardcoded for testing r^2

%% Initial and final conditions

% Velocity and position of the vehicle as initial constraints
model.xinitidx = 4:9;


% Define solver options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.maxit = 800;    % Maximum number of iterations
codeoptions.printlevel = 2; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;



%% Generate forces solver
FORCES_NLP(model, codeoptions);

%% Call solver
% Set initial guess to start solver from:

x0i = model.lb+(model.ub-model.lb)/2;
x0=repmat(x0i',model.N,1);
problem.x0=x0;


% Set parameters with the final point and final velocity  
% desired pose [7.65,-55,3]
% desired velocity [0, 0, 0]
% central point of the no_fly_zone [-2.4, -36.5]
param = [7.65; -55; 3; 0.2741; -1.9811; 0; obst_x; obst_y; tx; ty; 0; 0];
%       [pfx   pfy  pfz  vxf     vyf   vzf cx cy  tx    ty    vtx vty]

problem.all_parameters=repmat(param, model.N,1);

% Set initial constraint.   This is usually changing in the simulation to use
% the receding horizon
% initial pose [-18.8, -12.26]
% initial velocity [0, 0, 0]
problem.xinit = [initial_x; initial_y; initial_z; 0; 0; 0];

% Time to solve the NLP!
[output,exitflag,info] = FORCESNLPsolver(problem);

% Make sure the solver has exited properly.
%assert(exitflag == 1,'Some problem in FORCES solver');
fprintf('\nFORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);

%% Plot results
TEMP = zeros(model.nvar,model.N);
for i=1:model.N
    if(model.N>=100)
        if (i<100)
          TEMP(:,i) = output.(['x0',sprintf('%02d',i)]);
        else
          TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
        end
    else
      TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
    end

end

%% plotting output
u_x = TEMP(1,:);
u_y = TEMP(2,:);
u_z = TEMP(3,:);
x = TEMP(4,:);
y = TEMP(5,:);
z = TEMP(6,:);
v_x = TEMP(7,:);
v_y = TEMP(8,:);
v_z = TEMP(9,:);


plot(x,y, 'LineWidth', 3); hold on
hold on
xlabel('x (m)'); ylabel('y (m)');  zlabel('z (m)');


%% plotting the real no fly zone
% clear x
% x = -13.1:0.1:-2.5;
% 
% y = -1.5802*x-55.25; % recta arriba
% hold on
% plot(x,y)
% clear x
% x = -2.2:0.1:10.77;
% y = -1.457*x-24.01;
% hold on
% plot(x,y)
% 
% clear x
% x = -13.1:0.1:-2.2;
% y = 1.261*x-18.02;
% hold on
% plot(x,y)
% 
% clear x
% x = -2.5:0.1:10.7;
% y = 0.8742*x-49.11;
% hold on
% plot(x,y)

circle(obst_x,obst_y,radius)
hold on 
%circle(-2.4,-36.5,7)

hold on
initial_point = [initial_x initial_y]; 
final_point = [7.65 -55];
to_plot= [initial_point; final_point];
%plot(to_plot(:,1),to_plot(:,2),'b--');

hold on
plot(tx,ty,'rx')

hold on
plot(final_pose_x,final_pose_y,'bx', 'MarkerSize',10)

wo=load('trajectory_wo.mat');

plot(wo.TEMP(4,:),wo.TEMP(5,:),'g--');
% %%%%%%% calculating yaw diff
% yaw_diff=[];
% % vector angle sum
% for i= 2:49
%     previous_yaw = atan2((ty-y(i-1)),(tx-x(i-1)));
%     yaw = atan2((ty-y(i)),(tx-x(i)));
%     yaw_diff=[yaw_diff;yaw-previous_yaw];
% end
% suma = sum(yaw_diff)
% 
% clear previous_yaw yaw_diff yaw
% yaw_diff_wo = [];
% for j= 2:49
%     previous_yaw = atan2((ty-wo.TEMP(5,j-1)),(tx-wo.TEMP(4,j-1)));
%     yaw = atan2((ty-wo.TEMP(5,j)),(tx-wo.TEMP(4,j)));
%     yaw_diff_wo=[yaw_diff_wo;yaw-previous_yaw];
% end
% sum_wo = sum(yaw_diff_wo)

title('TOP VIEW - FLYOVER')
legend('cinematography term trajectory', 'No fly zone','Target','Desired point','without cinematography term')

