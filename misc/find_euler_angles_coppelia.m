function find_euler_angles_coppelia()
close all

pd = 1*[-1 -1 0]';
%pd = 1*[0 0 5]';


x = pd(1);
y = pd(2);
z = pd(3);

r = sqrt(x^2 + y^2 + z^2);
phi = atan2(y, x);
th = acos(z/r);


% giro sobre Z
rad2deg(phi)

% giro sobre Y
rad2deg(th)

%moving on local axes
Rz = Rot(phi, 'z');
Ry = Rot(th, 'y');
R = Rz*Ry
T = [R pd];
figure, 
grid
hold
axis equal
draw_axes(eye(4), 'X0', 'Y0', 'Z0', 2)
draw_axes(T, 'X', 'Y', 'Z', 2)

[alpha, beta, gamma] = rot2euler(R, 'XYZ');
'alpha deg'
rad2deg(alpha)
'beta deg'
rad2deg(beta)
'gamma deg'
rad2deg(gamma)

% function [alpha, beta, gamma] = rot2eul(R, convention)
% 
% 
% if convention=='XYZ'
%     [alpha, beta, gamma]=conventionXYZ(R);
%     return
% end
% 
% 
% function [alpha, beta, gamma] = conventionXYZ(R)
% %'R(1,3)=sen(beta)=1??'
% if abs(R(1,3)) == 1
%     % degenerate case in which sen(beta)=+-1 and cos(beta)=0
%     alpha = 0; % arbitrarily set alpha to zero
%     beta = asin(R(1,3));
%     gamma = atan2(R(2,2), R(2,1));    
% else
%     % standard way to compute alpha beta and gamma
%     alpha = -atan2(R(2,3), R(3,3));
%     gamma = -atan2(R(1,2), R(1,1));
%     beta = atan2(cos(alpha)*R(1,3), R(3,3));
% end

