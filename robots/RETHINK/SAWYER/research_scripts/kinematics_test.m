% test_kinematics

% Copyright (C) 2017, by Arturo Gil Aparicio
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
% along with ARTE.  If not, see <http://www.gnu.odrg/licenses/>.
close all;

fprintf('\nThe demo shows how to compute the end effectors speed as a function of the joint speeds and viceversa')
%
%robot=load_robot('RETHINK','SAWYER');

%First compute the linear and angular speeds of the end effector given the
%joint speeds qd = [2 2 2 2 2 2 2] rad/s.
% Compute at joint position q = [0.1 0.1 0.1 0.1 0.1 0.1] rad
%q = [0.0 0.1 0.1 0.1 0.1 0.1 0.1]'; 
q = [0.0 -0.7 0.0 2 -0.05 0.17 3.13]';
drawrobot3d(robot, q)
T = directkinematic(robot, q)
%The jacobian in the base reference system.
J = manipulator_jacobian(robot, q)


%Change any vector to the base reference system by: V0=TVn



