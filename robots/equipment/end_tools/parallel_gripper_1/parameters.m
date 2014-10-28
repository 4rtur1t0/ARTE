%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   vacuum tool.
%   PERS tooldata gripper:=[TRUE,[[0,0,425],[1,0,0,0]],[2.5,[0,0,200],[1,0,0,0],0.5,0.5,0.5]];
%   gripper=[TRUE,[[0,0,0.425],[1,0,0,0]],[2.5,[0,0,0.200],[1,0,0,0],0.5,0.5,0.5]];
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   09/01/2012
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
function robot = parameters()

robot.name= 'PARALLEL GRIPPER';

%Path where everything is stored for this robot
%robot.path = 'robots/end_tools/vacuum_1';

robot.DH.theta= '[]';
robot.DH.d='[]';
robot.DH.a='[]';
robot.DH.alpha= '[]';
robot.J=[];


robot.inversekinematic_fn = '';

%number of degrees of freedom
robot.DOF = 0;

%rotational: 0, translational: 1
robot.kind=[];

%minimum and maximum rotation angle in rad
robot.maxangle =[]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 0; %m/s

%base reference system
robot.T0 = eye(4);

%definition of the tool center point with respect to the last reference
%system.
%for tools, this TCP usually means the transformation from system 
%(X_tool0,Y_tool0,Z_tool0) to (X_tool1,Y_tool1,Z_tool1)
robot.TCP = [1 0 0 0;
             0 1 0 0;
             0 0 1 0.120;
             0 0 0 1]; 

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;
%needed by the simulation. Consider that the piece is not yet gripped (gripped = 0)
robot.piece_gripped=0;
%consider that the tool is closed (open=0)
robot.tool_open=0;




% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [100 102 100]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.75 0.75 -0.75 0.75 0 1.2];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;