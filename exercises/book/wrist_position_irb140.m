% A SYMBOLIC DERIVATION OF THE JACOBIAN MANIPULATOR OF A KUKA LBR IIWA 14
%
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
function wrist_position_irb140
% link lengths

syms q1 q2 q3 q4
robot = load_robot('ABB', 'IRB140')

d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

% matrices DH
A01 = dh_sym(q1, d(1), a(1), alpha(1));
A12 = dh_sym(q2-pi/2, d(2), a(2), alpha(2));
A23 = dh_sym(q3, d(3), a(3), alpha(3));
A34 = dh_sym(q4, d(4), a(4), alpha(4));

% wrist position
A04 = A01*A12*A23*A34;
A04 = simplify(A04)

% wrist position
pm = A04(1:3,4)




function A = dh_sym(theta, d, a, alpha)
syms q1 q2 q3 q4 q5 q6 q7
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
        

        
