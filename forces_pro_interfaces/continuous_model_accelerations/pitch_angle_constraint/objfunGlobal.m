function J = objfunGlobal( z,p)
%OBJFUN Summary of this function goes here

% minimize the accelerations ||u{k}||^2
% minimize the derivative of x-y angle of the vector that is pointing to
% the target in global coordinate system

% weights
cinematography_term = 1;
w1 = cinematography_term;
accelerations = 1;
w2 = accelerations;

J = 1000*(z(1)^2+z(2)^2+z(3)^2) +10000*(z(6)-3)^2 ;%+w1*(((z(8)-p(12))*(z(4)-p(9))-(z(5)-p(10))*(z(7)-p(11)))^2)/(((z(4)-p(9))^2+(z(5)-p(10))^2)); % cinematography term
                                


%% z = [ax ay az px py pz vx vy vz]  => [control states]
%  z =  1  2  3  4  5  6  7  8  9
% p=[pfx pfy pfz vxf vyf vzf cx cy tx ty vtx vty]
%p= [1    2   3   4   5   6  7  8   9 10 11 12 ]


end

