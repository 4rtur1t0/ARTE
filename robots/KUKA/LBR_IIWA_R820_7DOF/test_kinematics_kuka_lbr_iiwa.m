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

close all

% test both solutions: based on the transpose an on the Moore-Penrose
q0 = [0.1 -pi/2 -pi/2 0.1 0.1 0.1 0.1]';
fprintf('\nSimple test: try to reach T_target')
T_target = directkinematic(robot, 0*[0.1 0.1 0.1 0.1 0.1 0.1 0.1]);

qinv = inversekinematic(robot, T_target, q0);

T_reached = directkinematic(robot, qinv);
'diff T-T_target'
T_reached-T_target

% Plot REACHED SOLUTION
drawrobot3d(robot, qinv)
