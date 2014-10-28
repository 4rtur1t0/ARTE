%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   INIT_SIM_VARIABLES initializes the variables needed
%   during the simulations
%
%	See also LOAD_ROBOT
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. email:
%   arturo.gil@umh.es date:   02/01/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function robot = init_sim_variables(robot)

robot.debug=0;

%POSITION, velocity and acceleration
robot.q= zeros(robot.DOF, 1);
robot.qd=zeros(robot.DOF, 1);
robot.qdd=zeros(robot.DOF, 1);
robot.time = [];

robot.q_vector=[];
robot.qd_vector=[];
robot.qdd_vector=[];

robot.last_target=directkinematic(robot, robot.q);
robot.last_zone_data = 'fine';

robot.tool0=[];
robot.tool0=[1,[[0,0,0]/1000,[1,0,0,0]],[0.1,[0,0,0],[1,0,0,0],0,0,0]];
robot.wobj0=[];

%This is used of end tools
% for example, for a gripper, tool_activated grabs the piece
robot.tool_activated=0;

%save robot path
robot.path = pwd;
