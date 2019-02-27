function J = objfunN( z,p )
%OBJFUN Summary of this function goes here
%   Detailed explanation goes here

J =(p(1)-z(4))^2+(p(2)-z(5))^2+(p(3)-z(6))^2+(p(4)-z(7))^2+(p(5)-z(8))^2+(p(6)-z(9))^2; %minimize the difference between the current position and the desired position
% final position and velocities as parameters

end

