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
function direct_kinematics_symbolic_IRB140
% link lengths

syms q1 q2 q3 q4 q5 q6
%robot = load_robot('UR', 'UR5')

%d = eval(robot.DH.d);
%a = eval(robot.DH.a);
%alpha = eval(robot.DH.alpha);

% matrices DH
A01 = dh_sym(q1, 0.352, 0.07, -pi/2)
A10 = inv(A01)
A10 = simplify(A10)


 
 




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
        

        
