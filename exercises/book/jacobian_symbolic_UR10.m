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
function jacobian_symbolic_UR10
% link lengths

syms q1 q2 q3 q4 q5 q6
robot = load_robot('UR', 'UR10_coppelia')

d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

% matrices DH
A01 = dh_sym(q1, d(1), a(1), alpha(1));
A12 = dh_sym(q2-pi/2, d(2), a(2), alpha(2));
A23 = dh_sym(q3, d(3), a(3), alpha(3));
A34 = dh_sym(q4+pi/2, d(4), a(4), alpha(4));
A45 = dh_sym(q5, d(5), a(5), alpha(5));
A56 = dh_sym(q6, d(6), a(6), alpha(6));

A02 = A01*A12;
A03 = A02*A23;
A04 = A03*A34;
A05 = A04*A45;
A06 = A05*A56;
 
z0 = [0 0 1]';
z1 = A01(1:3,3);
z2 = A02(1:3,3);
z3 = A03(1:3,3);
z4 = A04(1:3,3);
z5 = A05(1:3,3);
% simplify expressions
z4 = simplify(z4);
z5 = simplify(z5);

% Jacobian in
Jw = [z0 z1 z2 z3 z4 z5];


 
p06=A06(1:3,4);
p16=A06(1:3,4)-A01(1:3,4);
p26=A06(1:3,4)-A02(1:3,4);
p36=A06(1:3,4)-A03(1:3,4);
p46=A06(1:3,4)-A04(1:3,4);
p56=A06(1:3,4)-A05(1:3,4);

Jv = [cross(z0, p06) cross(z1, p16) cross(z2, p26) cross(z3, p36) cross(z4, p46) cross(z5, p56) ];

Jw = simplify(Jw)
Jv = simplify(Jv)
% singularities = det(Jv)
Js = [Jv; Jw];

q1 = 0.1
q2 = 0.1
q3 = 0.1
q4 = 0.1
q5 = 0.1
q6 = 0.1
q = [.1 .1 .1 .1 .1 .1]
A01 = eval(A01);
A12
A12 = eval(A12)
A23 = eval(A23);
A34 = eval(A34);
A45 = eval(A45);
A56 = eval(A56);

a01 = dh(robot,q, 1) 
a12 = dh(robot,q, 2) 
a23 = dh(robot,q, 3) 
a34 = dh(robot,q, 4) 
a45 = dh(robot,q, 5) 
a56 = dh(robot,q, 6) 

A01-a01
A12-a12
A23-a23
A34-a34
A45-a45
A56-a56

Js = eval(Js)

J = manipulator_jacobian(robot, q)

J-Js

Ts = eval(A06)
T = directkinematic(robot, q)







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
        

        
