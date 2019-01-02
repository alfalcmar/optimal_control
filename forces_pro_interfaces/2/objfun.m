function J = objfun( z,p )
%OBJFUN Summary of this function goes here
%   Detailed explanation goes here

J = z(1)^2+z(2)^2+z(3)^2;%(p(1)-z(4))^2+(p(2)-z(5))^2+(p(3)-z(6))^2;

end

