function J = objfunN( z,p )
%OBJFUN Summary of this function goes here
% This desired position and desired velocity is given by the Executer shot
% module

% || x_{n} - [desired_position desired_velocity]' || ^2

% desired position and velocities as parameters

J =(p(1)-z(4))^2+(p(2)-z(5))^2+(p(3)-z(6))^2+(p(4)-z(7))^2+(p(5)-z(8))^2+(p(6)-z(9))^2; 


end

