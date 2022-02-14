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
function jacobian_symbolic_spherical_v
% link lengths
L3 = 0.38;
L6 = 0.065;

syms q1 q2 q3
% matrices DH
A01 = dh_sym(q1, L3, 0, pi/2);
A12 = dh_sym(q2, 0, 0, -pi/2);
A23 = dh_sym(q3, L6, 0, 0);

A02 = A01*A12;
A03 = A01*A12*A23;

z0 = [0 0 1]';
z1 = A01(1:3,3);
z2 = A02(1:3,3);

p03=A03(1:3,4);
p13=A03(1:3,4)-A01(1:3,4);
p23=A03(1:3,4)-A02(1:3,4);

Jv = [cross(z0, p03) cross(z1, p13) cross(z2, p23)];

singularities = det(Jv)



function A = dh_sym(theta, d, a, alpha)
syms q1 q2 q3
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
            0              sa             ca             d;
            0         0                     0              1];
