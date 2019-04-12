% This example solves the motion planning problem for a drone with the
% following simplified model:
%
%    dx/dt = v_x
%    dy/dt = v_y
%    dz/dt = v_z
%    dv_x/dt = a_x
%    dv_y/dt = a_y
%    dv_z/dt = a_z
%
% Variables are collected into z = [a_x a_y a_z x y z v_x v_y v_z].

clear; clc; close all;

%% Problem dimensions
% Nv = 1;                % number of vehicles
model.N = 70;            % horizon length
model.nvar = 9;         % number of variables
model.neq  = 6;          % number of equality constraints
model.nh = 1;            % number of inequality constraints 

%% Objective function 
% model.objective = @ objfun;
w1 = 10^4;
w2 = 2*10^1;
% g = 9.8;
model.objective = @(z)  w1*(z(1))^2 + w1*z(2)^2 + w1*z(3)^2 + 10*(z(6)-2)^2 + ...
                        100000*((z(4)*z(6)*z(7) + z(5)*z(6)*z(8) - z(9)*(z(4)^2 + z(5)^2))^2/((z(4)^2 + z(5)^2 + z(6)^2)^2*(z(4)^2 + z(5)^2)));
%% Dynamics, i.e. equality constraints 

integrator_stepsize = 2;
m = 1;     % kg
g = 9.8;   % m/s^2
continuous_dynamics = @(x,u) [x(4);  
                              x(5);  
                              x(6); 
                              u(1);
                              u(2);
                              u(3)];                                                       
                  
model.eq = @(z) RK4(z(4:9), z(1:3), continuous_dynamics, integrator_stepsize); % Using CasADi RK4 integrator

model.E = [zeros(6,3) eye(6)];

%% Inequality constraints
% upper/lower variable bounds on all variables : lb <= z <= ub
model.lb = [-1 -1 -1 -100 -100  0  -2  -2  -1];
model.ub = [ 1 +1 +1 +100 +100 +30  2   2   1];
                 
%% Distance from the target 
r = 2;
model.ineq = @(z)(z(4)-0)^2 + (z(5)-0)^2;                
model.hu = +1000;
model.hl = r^2;

%% Initial and final conditions

% Initial condition on the state vector
model.xinit = [-15; -15; 2];% 0.0 ; 0.0; 0.0];
model.xinitidx =  4:6;

% Final condition on the state vector
model.xfinal = [15; 15; 2; 0]; 
model.xfinalidx = [4:6, 9];

%% Define solver options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.maxit = 800;    % Maximum number of iterations
codeoptions.printlevel = 2; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;

% change this to your server or leave uncommented for using the standard
% embotech server at https://www.embotech.com/codegen
% codeoptions.server = 'http://yourforcesserver.com:8114/v1.5'; 

%% Generate forces solver
FORCES_NLP(model, codeoptions);

%% Call solver
% Initial guess for the solver:
x0i = model.lb + (model.ub - model.lb)/2 + 1;
x0=repmat(x0i', model.N, 1);
problem.x0=x0; 

% Set initial and final conditions. This can be changed in different instances of the problem:
problem.xinit = model.xinit;
problem.xfinal = model.xfinal;

% Time to solve the NLP!
[output,exitflag,info] = FORCESNLPsolver(problem);

% Make sure the solver has exited properly. 
assert(exitflag == 1,'Some problem in FORCES solver');
fprintf('\nFORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);

%% Plot results
TEMP = zeros(model.nvar,model.N);
for i=1:model.N
    TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
end
a_x = TEMP(1,:);
a_y = TEMP(2,:);
a_z = TEMP(3,:);
% Psi = TEMP(4,:);
x = TEMP(4,:);
y = TEMP(5,:);
z = TEMP(6,:);
v_x = TEMP(7,:);
v_y = TEMP(8,:);
v_z = TEMP(9,:);
% pt_x = TEMP(10,:);
% pt_y = TEMP(11,:);
% T = TEMP(10,1);
p_O = [0, 0, 0];


figure(1)
plot3(x,y,z, 'LineWidth', 3); hold on
plot3(model.xinit(1),model.xinit(2),model.xinit(3), 'rx', 'markers', 14); hold on    
xlabel('x (m)'); ylabel('y (m)');  zlabel('z (m)');
plot3(p_O(1),p_O(2),p_O(3), 'k.', 'markers', 14); hold on  
circle(0, 0, r);
grid
% plot3(pt_x,pt_y,zeros(size(pt_x)),'r.')
% plot3(4*pt_x,pt_y,4*ones(size(pt_x)),'b.')


ax = gca();
newfig = figure(2);
az = 0;
el = 90;
newax = copyobj(ax, newfig);
view(newax, [az, el]);
title('Top view')


