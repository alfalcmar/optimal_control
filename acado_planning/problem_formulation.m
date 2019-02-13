BEGIN_ACADO  %% every problem should start with BEGIN_ACADO

acadoSet('problemname', 'trajectory_generator');

DifferentialState p_x p_y p_z v_x v_y v_z;
Control u_x u_y u_z;
%%%%%% MODEL%%%%%%%%%%%%%%%%%
h=1; %time_step
f = acado.DiscretizedDifferentialEquation(h);

f.add(next(p_x) == p_x + h*v_x + h^2/2*u_x);
f.add(next(p_y) == p_y + h*v_y + h^2/2*u_y);
f.add(next(p_z) == p_z + h*v_z + h^2/2*u_z);

%%%%%% OPTIMIZATION PROBLEM %%%%%%%%%%%%%%%%%%
tStart=0;
tEnd = 30;
N = 30;
ocp = acado.OCP(tStart, tEnd, N); % set up the OCP. Start at tStart, control in tEnd, Interval = N

ocp.minimizeMayerTerm(u_x); % Minimize a Mayer term


%%%%%% Constraints

ocp.subjectTo( f ); % Your OCP is always subject to your 
% initial condition (initial pose)
 ocp.subjectTo( 'AT_START', p_x == 1.0 );  
 ocp.subjectTo( 'AT_START', p_y == 0.0 );
 ocp.subjectTo( 'AT_START', p_z == 0.0 );
  ocp.subjectTo( 'AT_START', v_x == 0.0 );  
 ocp.subjectTo( 'AT_START', v_y == 0.0 );
 ocp.subjectTo( 'AT_START', v_z == 0.0 );
 
% % final condition
 ocp.subjectTo( 'AT_END', p_x == 5.0 );  
 ocp.subjectTo( 'AT_END', p_y == 5.0 );
 ocp.subjectTo( 'AT_END', p_z == 5.0 );
 
  ocp.subjectTo( 'AT_END', v_x == 0.0 );
 ocp.subjectTo( 'AT_END', v_y == 0.0 );
 ocp.subjectTo( 'AT_END', v_z == 0.0 );

 
 ocp.subjectTo(-1<= v_x <=1); 
 ocp.subjectTo(-1<= v_y <=1); 
 ocp.subjectTo(-1<= v_z <=1); 
 
 ocp.subjectTo(-0.5<= u_x <=0.5); 
 ocp.subjectTo(-0.5<= u_y <=0.5); 
 ocp.subjectTo(-0.5<= u_z <=0.5); 
 
 


 %% Optimization Algorithm
 algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
   % algo.set('KKT_TOLERANCE', 1e-5 );        % Set some parameters for the algorithm
%algo.set('RELAXATION_PARAMETER', 1.5 );
%algo.set('MAX_NUM_STEPS', 500);


END_ACADO

out = trajectory_generator_RUN(); % Run the test. The name of the RUN file

%% call the solver
%out = trajectory_generator_RUN();

