function J = objfun( z,p)
%OBJFUN Summary of this function goes here

% minimize the accelerations ||u{k}||^2
% minimize the derivative of x-y angle of the vector that is pointing to
% the target in global coordinate system

% weights
cinematography_term = 100;
w1 = cinematography_term;
accelerations = 0.005;
w2 = accelerations;

J = w2*(z(1)^2+z(2)^2+z(3)^2)+w1*((atan2((z(4)*z(8)-z(5)*z(7)-p(9)*z(8)+p(10)*z(7)),(p(9)*z(7)-z(4)*z(7)-z(5)*z(8)+p(10)*z(8)))-...
                              atan2(z(10)*z(14)-z(11)*z(13)-p(13)*z(14)+p(14)*z(13),(p(13)*z(13)-z(10)*z(13)-z(11)*z(14)+p(14)*z(14))))^2);

%% z = [ax ay az px py pz vx vy vz px-1 py-1 pz-1 vx-1 vy-1 vz-1]  => [control states]
%  z =  1  2  3  4  5  6  7  8  9   10  11    12   13   14   15
% p=[pfx pfy pfz vxf vyf vzf cx cy tx ty vtx vtz tx-1 tx-1]
%p= [1    2   3   4   5   6  7  8   9 10 11 12  13    14]


end

