% SCRIPT TEST FOR THE KINEMATIC PROBLEM FOR SERIAL ROBOTS

% Copyright (C) 2012, by Arturo Gil Aparicio
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
% 
% qinv =
% 
%          0         0         0         0    3.1416    3.1416    3.1416    3.1416
%    -0.0412   -0.0412    1.6597    1.6597   -1.5798   -1.5798   -0.3550   -0.3550
%     0.0720    0.0720    3.0696    3.0696   -0.3825   -0.3825   -2.7591   -2.7591
%     0.0000   -3.1416    3.1416   -0.0000    0.0000    3.1416    0.0000   -3.1416
%    -0.0308    0.0308   -1.5539    1.5539   -1.1793    1.1793   -0.0275    0.0275
%    -0.0000    3.1416   -3.1416         0    3.1416   -0.0000    3.1416   -0.0000

function test_algebraic_irb140

close all

fprintf('\nTHE DEMO PRESENTS THE DIRECT AND INVERSE KINEMATIC PROBLEM')

%load robot parameters. You can try different robots%
robot=load_robot('ABB', 'IRB140'); 
%adjust 3D view as desired
adjust_view(robot)

q = [0.1 0.1 0.1 0.1 0.1 0.1];
T = directkinematic(robot, q)
% reach this!
%T =[0.0000   0.0000    1.0000    0.4000;
%   0.0000    1.0000    0.0000   0.0000;
%   -1.0000   0.0000    0.0000    0.6000;
%   0         0         0    1.0000];

%Call the inversekinematic for this robot. All the possible solutions are
%stored at qinv. At least, one of the possible solutions should match q
qinv = inversekinematic(robot, T)
for i=1:8
    drawrobot3d(robot, qinv(:, i))
end

[q1, q2, q3] = algebraic(robot, T)

drawrobot3d(robot, [q1 q2 q3 0 0 0])

function [q1, q2, q3] = algebraic(robot, T)
p0 = T(1:3, 4);
pm0 = p0 - T(1:3,3)*0.065;

d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha= eval(robot.DH.a);
L1=d(1);
L2=a(1);
L3=a(2);
L4=d(4);

q1 = atan2(pm0(2), pm0(1));
A01 = dh(robot, [q1 0 0 0 0 0], 1);
pm1 = inv(A01)*[pm0; 1];

R1 = sqrt(pm1(1)^2 + pm1(2)^2);
alpha = atan2(pm1(2), pm1(1));

%q2 = asin((R1^2+L3^2-L4^2)/(2*L3*R1))-alpha;
q2 = alpha-asin((L4^2-R1^2-L3^2)/(2*L3*R1));
q3 = asin((L3^2+L4^2-R1^2)/(2*L3*L4));




