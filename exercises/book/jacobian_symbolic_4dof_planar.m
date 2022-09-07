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
function jacobian_symbolic_4dof_planar

syms q1 q2 q3 q4
% matrices DH
A01 = dh_sym(q1, 0, 0.4, 0);
A12 = dh_sym(q2, 0, 0.3, 0);
A23 = dh_sym(q3, 0, 0.2, 0);
A34 = dh_sym(q4, 0, 0.1, 0);
A02 = A01*A12;
A03 = A01*A12*A23;
A04 = A01*A12*A23*A34;
A04 = simplify(A04);

z0 = [0 0 1]';

p04=A04(1:3,4);
p14=A04(1:3,4)-A01(1:3,4);
p24=A04(1:3,4)-A02(1:3,4);
p34=A04(1:3,4)-A03(1:3,4);

Jv = [cross(z0, p04) cross(z0, p14) cross(z0, p24) cross(z0, p34)];
Jv = simplify(Jv)

Jw = [z0 z0 z0 z0];
J = [Jv; Jw]
% transform to valid dimensions
J = [J(1:2,:); J(6,:)]

J = simplify(J)
%singularities = det(J*J')
%singularities = simplify(singularities)

%solve(singularities==0)

% compute moore-penrose pseudo inverse
Jp = J'*inv(J*J')
Jp = simplify(Jp)

% compute projector P
P = eye(4)-Jp*J





function A = dh_sym(theta, d, a, alpha)
syms q1 q2 q3 q4
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
