function J = objfunGlobal( z,p)
%OBJFUN Summary of this function goes here

% minimize the accelerations ||u{k}||^2
% minimize the derivative of x-y angle of the vector that is pointing to
% the target in global coordinate system
epsilon = 0.001;
% weights

w1 = 1;
w2 = 10000;
w3 = 1;
w4 = 10000;

J = w1*(z(1)^2+z(2)^2+z(3)^2) + w2*(z(6)-3)^2+... % without varying height
                                w3*(((z(8)-p(12))*(z(4)-z(10))-(z(5)-z(11))*(z(7)-p(11)))^2)/(((z(4)-z(10))^2+(z(5)-z(11))^2)+epsilon) + ... % YAW
                                w4*(((z(4)-z(10))*z(6)*(z(7)-p(11)) + (z(5)-z(11))*z(6)*(z(8)-p(12)) - z(9)*((z(4)-z(10))^2 + (z(5)-z(11))^2))^2/(epsilon+((z(4)-z(10))^2 + (z(5)-z(11))^2 + z(6)^2)^2*(epsilon+(z(4)-z(10))^2 + (z(5)-z(11))^2))); % PITCH
                                


%% z = [ax ay az px py pz vx vy vz tx ty]  => [control states]
%  z =  1  2  3  4  5  6  7  8  9  10 11
% p=[pfx pfy pfz vxf vyf vzf cx cy tx ty vtx vtz]
%p= [1    2   3   4   5   6  7  8   9 10 11 12 ]


end

