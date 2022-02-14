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
function direct_kinematics_symbolic_KUKA_LBR
% link lengths

syms q1 q2 q3 q4 q5 q6 Q7
robot = load_robot('KUKA', 'LBR_IIWA_R820_7DOF')

d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

% matrices DH
A01 = dh_sym(q1, d(1), a(1), alpha(1));
A12 = dh_sym(q2, d(2), a(2), alpha(2));
A23 = dh_sym(q3, d(3), a(3), alpha(3));
A34 = dh_sym(q4, d(4), a(4), alpha(4));
A45 = dh_sym(q5, d(5), a(5), alpha(5));
A56 = dh_sym(q6, d(6), a(6), alpha(6));
A67 = dh_sym(q7, d(7), a(7), alpha(7));

A02 = A01*A12;
A03 = A02*A23;
A04 = A03*A34;
A05 = A04*A45;
A06 = A05*A56;
A07 = A06*A67;

A07 = simplify(A07)
 
A03 = simplify(A03)




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
        

        
