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
function direct_kinematics_symbolic_KUKA_KR5
% link lengths

syms q1 q2 q3

robot = load_robot('KUKA', 'KR5_sixx_R650')


robot.DH.theta= '[-q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)]';
robot.DH.d='[-0.335 0 0 -0.295 0 0.080]';
robot.DH.a='[0.075 0.270 0.090 0 0 0]';
robot.DH.alpha= '[pi/2 0 pi/2 -pi/2 -pi/2 pi]';

d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

% matrices DH
A01 = dh_sym(-q1, d(1), a(1), alpha(1));
A12 = dh_sym(q2-pi/2, d(2), a(2), alpha(2));
A23 = dh_sym(q3, d(3), a(3), alpha(3));

A02 = A01*A12;
A03 = A02*A23;


A03 = simplify(A03)

q1 = 0
q2 = pi/2
q3 = -pi/2
A03 = eval(A03)
 
 




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
        

        
