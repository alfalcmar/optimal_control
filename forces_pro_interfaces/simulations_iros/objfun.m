function J = objfun( z,p )
%OBJFUN Summary of this function goes here
%   Detailed explanation goes here

J = z(1)^2+z(2)^2+z(3)^2; %minimize the accelerations

end

