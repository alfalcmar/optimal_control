function J = objfunGlobal( z,p)
%OBJFUN Summary of this function goes here

% minimize the accelerations ||u{k}||^2
% minimize the derivative of x-y angle of the vector that is pointing to
% the target in global coordinate system

% weights
cinematography_term = 1000;
w1 = cinematography_term;
accelerations = 1;
w2 = accelerations;

J = w2*(z(1)^2+z(2)^2+z(3)^2) + w1*(((p(12)-z(8))*(z(10)-z(4))-(z(11)-z(5))*(p(11)-z(7)))^2)/(((z(4)-z(10))^2+(z(5)-z(11))^2)); %cinematography term yaw derivative
% w0*pith_derivative_cinematography_term //TO INCLUDE
                                


%% z = [ax ay az px py pz vx vy vz tx ty]  => [control states]
%  z =  1  2  3  4  5  6  7  8  9  10 11
% p=[pfx pfy pfz vxf vyf vzf cx cy tx ty vtx vtz]
%p= [1    2   3   4   5   6  7  8   9 10 11 12 ]


end

