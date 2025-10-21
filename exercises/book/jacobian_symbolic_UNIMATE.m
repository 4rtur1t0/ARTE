% Copyright (C) 2016, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function jacobian_symbolic_UNIMATE
% link lengths

syms q1 q2 q3 q4 q5 L0 L1
robot = load_robot('UNIMATE', 'UNIMATE1')

%d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

% matrices DH simbólicas
A01 = dh_sym(q1,     L0, a(1), alpha(1));
A12 = dh_sym(q2+pi/2, 0, a(2), alpha(2));
A23 = dh_sym(0,      q3+L1, a(3), alpha(3));

A02 = A01*A12;
A03 = A02*A23;
 
z0 = [0 0 1]';
z1 = A01(1:3,3);
z2 = A02(1:3,3);

% Jacobian in angular speed
Jw = [z0 z1 z2];
Jw = simplify(Jw)
 
p03=A03(1:3,4);
p13=A03(1:3,4)-A01(1:3,4);


% jacobian in linear speed, caution, z2, since q3 is prismatic
Jv = [cross(z0, p03) cross(z1, p13) z2];


Jv = simplify(Jv)
% % singularities = det(Jv)
% Js = [Jv; Jw];
% 
% q1 = 0.1
% q2 = 0.1
% q3 = 0.1
% q4 = 0.1
% q5 = 0.1
% q6 = 0.1
% q = [q1 q2 q3 q4 q5 q6]
% Js = eval(Js)
% J = manipulator_jacobian(robot, q)
% 
% J-Js
% 
% 






function A = dh_sym(theta, d, a, alpha)
syms q1 q2 q3 q4 q5 q6
% avoid almost zero elements in cos(alpha) and sin(alpha)
ca = cos(alpha);
sa = sin(alpha);
if abs(ca) < 1e-6
    ca = 0;
end
if abs(sa) < 1e-6
    sa = 0;
end


A=[cos(theta)  -ca*sin(theta)   sa*sin(theta)   a*cos(theta);
   sin(theta)   ca*cos(theta)  -sa*cos(theta)   a*sin(theta);
            0         sa             ca             d;
            0         0               0             1];
        

        
