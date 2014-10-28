%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the KUKA KR30_jet.
%   
%   Authors: Sir Juan Carlos Blay, Sir Francisco Manuel Sabuco, Mr. Fernando Torres and Lord Emilio López.
%   Universidad Miguel Hernández de Elche. 
%   date:  20/12/2012
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

%Denavit-Hartemberg parameters
robot.DH.theta= '[deg2rad(20.34) q(2)+deg2rad(69.66) q(3) q(4) q(5) q(6)]';
robot.DH.d='[q(1) 0 0 -0.83 0 -0.15]';
robot.DH.a='[-0.486 0.85 0.145 0 0 0]';
robot.DH.alpha= '[0 0 pi/2 -pi/2 pi/2 pi]';

%Jacobian matrix
robot.J=[];

%robot name
robot.name= 'KUKA_KR30_jet';

%inverse kinematic function
robot.inversekinematic_fn = 'inversekinematic_kuka_kr30_jet(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%joint movement kind; rotational: 5, translational: 1
robot.kind=['T' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[-2.5 2.75; %Axis 1, minimum, maximum
                deg2rad(0) deg2rad(180); %Axis 2, minimum, maximum
                deg2rad(-120) deg2rad(158); %Axis 3
                deg2rad(-350) deg2rad(350); %Axis 4
                deg2rad(-119) deg2rad(150); %Axis 5
                deg2rad(-350) deg2rad(350)]; %Axis 6

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [3.2; %Axis 1, m/s
                deg2rad(126); %Axis 2, rad/s
                deg2rad(166); %Axis 3, rad/s
                deg2rad(260); %Axis 4, rad/s
                deg2rad(245); %Axis 5, rad/s
                deg2rad(322)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 3.2; %m/s
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
%base reference system
 robot.T0 = eye(4);
 
%             [-1 0 0 0;
%             0 0 1 0;
%             0 1 0 0;
%             0 0 0 1];
      
        
%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);

% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [202, 97, 0]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-2 3 -4 4 -4 4];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;