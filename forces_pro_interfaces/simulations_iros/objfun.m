function J = objfun( z,p )
%OBJFUN Summary of this function goes here

% minimize the accelerations ||u{k}||^2

J = z(1)^2+z(2)^2+z(3)^2; % + cinematography term

%% We have to include the cinematography term 
% Director vector pointing from drone to target r_k = p_k - p^{tar}
% Director vector expressed in drone coordinate system (D)
%{}_{D}r_k = Rot(yaw)*r_k
% Drone pointing forward, roll and pitch approximated to zero. 
% yaw = atan(v_k(1)/v_k(0))
% Compute pan and tilt angles of the gimbal from {}_{D}r_k
% Compute angular velocities

end

