% SCRIPT TO TEST THE GRAPHIC CAPABILITIES OF THE TOOLBOX: ROBOT IN A MANUFACTURING CELL

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
echo on

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q = [0 0 0 0 0 0];
% 
% THE DEMO PRESENTS THE ROBOT IN A ROBOTIC WELDING CELL
%
%  START BY LOADING LOAD A ROBOT BY MEANS OF THE LOAD_ROBOT FUNCTION'
% 
% robot=load_robot('abb', 'IRB140');
%
%  LOTS OF OTHER ROBOTS CAN BE LOADED, PLEASE TRY:
%
%   robot=load_robot('abb', 'IRB6620'); 
%   robot=load_robot('kuka', 'KR5_arc'); 
%   robot=load_robot('kuka', 'KR5_sixx_R650');
%   MORE ROBOTS AVAILABLE UNDER THE arte_lib3.x/robots DIRECTORY
%
%   Example:
%
% robot=load_robot('kuka', 'KR5_arc');
%
% adjust_view(robot);
%
% fprintf('\nNOW LOAD AUXILIAR EQUIPMENT')
% robot.equipment=load_robot('equipment', 'bodywork');
%
% drawrobot3d(robot, q)
% 
% fprintf('\nNOW LOAD A GRIPPER')
% robot.tool=load_robot('equipment/end_tools', 'spot_welding'); 
% 
% drawrobot3d(robot, q)
%  fprintf('\nNOW THE TEACHING PENDANT')
% teach
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

echo off

fprintf('\nLOAD A ROBOT:\n')

robot=load_robot('KUKA', 'KR5_2ARC_HW');


fprintf('\nNOW DRAW THE ROBOT')
drawrobot3d(robot, q)

adjust_view(robot);


fprintf('\nNOW LOAD AUXILIAR EQUIPMENT')
robot.equipment=load_robot('equipment', 'bodywork'); 

fprintf('\nNOW LOAD A GRIPPER')
robot.tool=load_robot('equipment/end_tools', 'spot_welding'); 


%draw the robot
drawrobot3d(robot, q)


teach





