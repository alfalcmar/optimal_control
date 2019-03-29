clear; clc; close all;
% change for your local path
addpath('/home/grvc/Desktop/casadi')
addpath('/home/grvc/Desktop/FORCES_PRO_CLIENT')
import casadi.*;

%% obstacle and target position
radius = 6;
tx = -8.4; 
ty = -29.5;


%% Problem dimensions
model.N = 30;            % horizon length
model.nvar = 9;          % number of variables
model.neq  = 6;          % number of equality constraints
model.nh = 1;            % number of inequality constraints
model.npar = 12;         % [pfx pfy pfz vxf vyf vzf cx cy tx ty tz vtx vtz]
                         % [1    2   3   4   5   6  7  8  9  10 11 12   13]
       
t= 0.1;  %% time step - integrator

% time step and horizon lenght to use the solver in the receding horizon in
% real experiments


% z1 : Torque
% z2 z3: pitch and roll angle
% z4 z5 z6 : x y z
% z7 z8 z9 : v_x v_y v_z
%% Objective function 

model.objective = @objfun;  
model.objectiveN = @objfunN; 

%% MODEL %%%%%%%%%%%%%%%
%% Continous model
integrator_stepsize = 0.1;
m = 1;     % kg
g = 9.8;   % m/s^2
continuous_dynamics = @(x,u) [x(4);  
                              x(5);  
                              x(6); 
                              u(1)*(cos(u(2))*sin(u(3))*cos(atan2(x(5),x(4))) + sin(u(2))*sin(atan2(x(5),x(4))))/m;
                              u(1)*(cos(u(2))*sin(u(3))*sin(atan2(x(5),x(4))) - sin(u(2))*cos(atan2(x(5),x(4))))/m;
                              -g +  u(1)*(cos(u(2))*cos(u(3)))/m;];
model.eq = @(z) RK4(z(4:9), z(1:3), continuous_dynamics, integrator_stepsize);

model.E = [zeros(6,3) eye(6)]; % A(6x6) B(6*3)

%% position, velocities and accelerations limits
% upper/lower variable bounds lb <= x <= ub
model.lb = [-15 -pi/6 -pi/6 -200 -200 0   -12 -12 -1];
model.ub = [+15 +pi/6 +pi/6 +200 +200 +50 12 12 3];

%% nonlinear inequalities
% (vehicle_x - obstacle_x)^2 +(vehicle_y - obstacle_y)^2 > r^2
model.ineq = @(z,p)  [(z(4)-p(7))^2 + (z(5)-p(8))^2]   %% obstacle position as parameter


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

x0i = model.lb + (model.ub - model.lb)/2 + 1;
x0=repmat(x0i', model.N, 1);
problem.x0=x0; 

% Set parameters with the final point and final velocity  
% desired pose [7.65,-55,3]
% desired velocity [0, 0, 0]
% central point of the no_fly_zone [-2.4, -36.5]
param = [7.65; -55; 3; 0.2741; -1.9811; 0; 0; 0; -8.4; -29.5; 0; 0];
%       [pfx   pfy  pfz  vxf     vyf   vzf cx cy  tx    ty    vtx vty]

problem.all_parameters=repmat(param, model.N,1);

% Set initial constraint.   This is usually changing in the simulation to use
% the receding horizon
% initial pose [-18.8, -12.26]
% initial velocity [0, 0, 0]
problem.xinit = [-18.8; -12.26; 3; 0; 0; 0];

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

circle(-2.4,-36.5,6)
hold on 
%circle(-2.4,-36.5,7)

hold on
initial_point = [-18.8 -12.26];
final_point = [7.65 -55];
to_plot= [initial_point; final_point];
plot(to_plot(:,1),to_plot(:,2),'b--');

hold on
plot(tx,ty,'rx')



