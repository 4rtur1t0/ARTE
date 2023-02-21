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
function spherical_wrist_irb140
% link lengths

syms q1 q2 q3 q4 q5 q6
robot = load_robot('ABB', 'IRB140')

d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

% matrices DH
A34 = dh_sym(q4, d(4), a(4), alpha(4));
A45 = dh_sym(q5, d(5), a(5), alpha(5));
A56 = dh_sym(q6+pi, d(6), a(6), alpha(6));

% spherical wrist
Q = A34*A45*A56;
Q = simplify(Q)

q5 = 0

Q = simplify(Q)


Q(1:3, 1:3)
 








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
        

        
