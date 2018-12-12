clear; clc; close all;

%% Problem dimensions
Nv = 1;
model.N = 30;            % horizon length
model.nvar = 9;          % number of variables
model.neq  = 6;          % number of equality constraints
model.nh = 1;            % number of inequality constraint functions

%% Objective function 

model.objective = @objfun; 

%% Dynamics, i.e. equality constraints 

t= 1;
A = [eye(3) t*eye(3); zeros(3) eye(3)];
B = [(t^2)/2*eye(3); t*eye(3)];

model.eq = @(z) [ A*[z(4);z(5);z(6);z(7);z(8);z(9)] + B*[z(1);z(2);z(3)]];

model.E = [zeros(6,3) eye(6)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
model.lb = [-0.2 -0.2 -0.2 -10 -10 0   0 0 0];
model.ub = [+0.2 +0.2 +0.2 +10 +10 +30 1 1 1];

% General (differentiable) nonlinear inequalities hl <= h(x) <= hu
model.ineq = @(z)  (z(4))^2 + (z(5))^2;
               
% Upper/lower bounds for inequalities
model.hu = +inf;
model.hl = 4;

%% Initial and final conditions

% Initial condition on drone states
model.xinitidx = 4:9;

% Final condition on drone velocity 
model.xfinalidx = 4:9;

%% Define solver options
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

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:
problem.xinit = [-5; -5; 5; 0; 0; 0];
problem.xfinal =[6; 6; 20; 0; 0; 0];

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
u_x = TEMP(1,:);
u_y = TEMP(2,:);
u_z = TEMP(3,:);
x = TEMP(4,:);
y = TEMP(5,:);
z = TEMP(6,:);
v_x = TEMP(7,:);
v_y = TEMP(8,:);
v_z = TEMP(9,:);


subplot (1,2,1)
plot3(x,y,z, 'LineWidth', 3); hold on
% axis equal

% plot(Y(:,2),X(:,2));
xlabel('x (m)'); ylabel('y (m)');  zlabel('z (m)');
grid
plot3(5,4,20, 'r.', 'markers', 10);
circle(0,0,2)

subplot(1,2,2)
plot3(x,y,z, 'LineWidth', 3); hold on

% plot(Y(:,2),X(:,2));
xlabel('x (m)'); ylabel('y (m)');  zlabel('z (m)');
grid
circle(0,0,2)

az = 0;
el = 90;
view(az, el);