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
function gimbal_DH
% A gimbal

syms q1 q2 q3


% matrices DH
A01 = dh_sym(q1, 0, 0, -pi/2);
A12 = dh_sym(q2-pi/2, 0, 0, pi/2);
A23 = dh_sym(q3, 0, 0, 0);

A02 = A01*A12;
A03 = A02*A23
A03 = simplify(A03)

q1 = 0.1
q2 = 0.2
q3 = 0.3

A01 = eval(A01)
A02 = eval(A02)
A03 = eval(A03)

figure, hold, grid
draw_axes(eye(4), 'X0', 'Y0', 'Z0', 1);
draw_axes(A01, 'X1', 'Y1', 'Z1', 1);
draw_axes(A02, 'X2', 'Y2', 'Z2', 1);
draw_axes(A03, 'X3', 'Y3', 'Z3', 1);
 
 




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
        

        
