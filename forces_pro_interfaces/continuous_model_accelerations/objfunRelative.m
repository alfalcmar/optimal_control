function J = objfunRelative( z,p)
%OBJFUN Summary of this function goes here

% minimize the accelerations ||u{k}||^2
% minimize the derivative of x-y angle of the vector that is pointing to
% the target in global coordinate system

% weights
cinematography_term = 400;
w1 = cinematography_term;
accelerations = 1; %20
w2 = accelerations;

J = w2*(z(1)^2+z(2)^2+z(3)^2)+ w1*(((((z(7)-p(11))*(z(5)-p(10))-(z(8)-p(12))*(z(4)-p(9)))/((z(5)-p(10))^2+(z(4)-p(9))^2))-...
                                                       ((z(2)*z(7)-z(1)*z(8))/(z(7)^2+z(8)^2)))^2);
                                                       
%J = w2*(z(1)^2+z(2)^2+z(3)^2)+ w1*((((p(12)-z(8))*(p(9)-z(4))-((p(11)-z(7))*(p(10)-z(5)))/((z(5)-p(10))^2+(z(4)-p(9))^2))-...
                                    %((z(2)*z(7)-z(1)*z(8))/(z(7)^2+z(8)^2)))^2);


%% z = [ax ay az px py pz vx vy vz]  => [control states]
%  z =  1  2  3  4  5  6  7  8  9
% p=[pfx pfy pfz vxf vyf vzf cx cy tx ty vtx vty]
%p= [1    2   3   4   5   6  7  8   9 10 11 12 ]


end

