% SCRIPT TEST FOR THE KUKA LBR ROBOT KINEMATICS

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
robot = load_robot('KUKA', 'LBR_IIWA_R820_7DOF');
close all

% test both solutions: based on the transpose an on the Moore-Penrose
q0 = pi/4*[1,1,1,1,1,1,1]';

fprintf('\nSimple test: try to reach T_target')
T_target = directkinematic(robot, q0)
drawrobot3d(robot, q0)

qinv = inversekinematic(robot, T_target, q0);

T_reached = directkinematic(robot, qinv);
'diff T-T_target'
T_reached-T_target

% Plot REACHED SOLUTION
drawrobot3d(robot, qinv)

J = manipulator_jacobian(robot, q0)
f = [1 0 0 0 0 0]'
tau = J'*f

f = pinv(J')*tau

mor = inv(J*J')*J

q0 = pi/4*[0 0 0 0 0 0 0]';
q0 = pi/4*[1,1,1,1,1,1,1]';
J = manipulator_jacobian(robot, q0)
[u, s, v]=svd(J')
f = [0 1 0 0 0 0]'
tau = J'*f



mor = pinv(J')

[u, s, v]=svd(mor)
f = pinv(J')*v(:,7)