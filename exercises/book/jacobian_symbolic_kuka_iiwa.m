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
function jacobian_symbolic_kuka_iiwa
% link lengths

syms q1 q2 q3 q4 q5 q6 q7
robot = load_robot('KUKA', 'LBR_IIWA_R820_COP')

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
 
z0 = [0 0 1]';
z1 = A01(1:3,3);
z2 = A02(1:3,3);
z3 = A03(1:3,3);
z4 = A04(1:3,3);
z5 = A05(1:3,3);
z6 = A06(1:3,3);

% simplify expressions
z2 = simplify(z2);
z3 = simplify(z3);
z4 = simplify(z4);
z5 = simplify(z5);
z6 = simplify(z6);



 
p07=A07(1:3,4);
p17=A07(1:3,4)-A01(1:3,4);
p27=A07(1:3,4)-A02(1:3,4);
p37=A07(1:3,4)-A03(1:3,4);
p47=A07(1:3,4)-A04(1:3,4);
p57=A07(1:3,4)-A05(1:3,4);
p67=A07(1:3,4)-A06(1:3,4);

% Jacobian in linear speed
Jv = [cross(z0, p07) cross(z1, p17) cross(z2, p27) cross(z3, p37) cross(z4, p47) cross(z5, p57) cross(z6, p67)];
% Jacobian in angular speed
Jw = [z0 z1 z2 z3 z4 z5 z6];

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
q7 = 0.1

Js = eval(Js)

q = [q1 q2 q3 q4 q5 q6 q7]
J = manipulator_jacobian(robot, q)

J-Js



Ts = eval(A07)
T = directkinematic(robot, q)

T-Ts







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
        

        
