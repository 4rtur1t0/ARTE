%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   AbsJPath
%   Moves robot to a particular joint coordinates%
% CAUTION: NOT TO BE CONFUSED WITH THE RAPID FUNCTIONS
% Once
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function [q_t, qd_t, qdd_t, time] = AbsJPath(robot, target_joint_coord, target_joint_speed, speed_percent)

%global configuration %  robot

fprintf('\nCall to AbsJPath ARTE');

%obtain current joint coordinates
q_initial=robot.q;
qd_initial = robot.qd;

%obtain target joint coordinates
q_final= target_joint_coord'; %joint_coord(1:robot.DOF)';
qd_final = target_joint_speed';

% compute max trapezoidal joint speed
qdmax = speed_percent*robot.velmax/100;
taccel = 0.1; %*ones(robot.DOF, 1); % seconds, parameter


%test joint limits for the final values
test_joints(robot, q_final);

% Perform a path planning on joints considering a trapezoidal speed profile
[q_t, qd_t, qdd_t, time]= compute_trapezoidal_profile(q_initial, qd_initial, q_final, qd_final, qdmax, taccel);



