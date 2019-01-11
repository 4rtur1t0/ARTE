%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB140.
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
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

robot.name= 'EPSON_FLEXION_N2_A450S';

robot.DH.theta= '[q(1)-pi/2 q(2)-pi/2 q(3)+pi/2 q(4) q(5) q(6)+pi/2]';
robot.DH.d='[0.3304 0 0 0.29 0 0.057]';
robot.DH.a='[0 -0.16 0 0 0 0]';
robot.DH.alpha= '[-pi/2 0 pi/2 -pi/2 pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_flexion_n2(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[-pi pi; %Axis 1, minimum, maximum
                 -pi pi; %Axis 2, minimum, maximum
                 -pi pi; %Axis 3
                deg2rad(-195) deg2rad(195); %Axis 4
                deg2rad(-130) deg2rad(130); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(297); %Axis 1, rad/s
                deg2rad(297); %Axis 2, rad/s
                deg2rad(356); %Axis 3, rad/s
                deg2rad(356); %Axis 4, rad/s
                deg2rad(360); %Axis 5, rad/s
                deg2rad(360)];%Axis 6, rad/s
    
robot.accelmax=robot.velmax/0.15; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 2.5; %m/s



%base reference system
robot.T0 = [-1 0 0 0; 0 -1 0 0; 0 0 -1 0.9; 0 0 0 1];


%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.5 0.75 -0.75 0.75 0 1.1];
%read graphics files
robot = read_graphics(robot);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC PARAMETERS
%   WARNING! These parameters do not correspond to the actual IRB 140
%   robot. They have been introduced to demonstrate the necessity of 
%   simulating the robot and should be used only for educational purposes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
%Base: 4kg
robot.dynamics.masses=[5 3 2.5 2.5 1.5 0.5];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[-0.19  0       0.1612; %(rx, ry, rz) link 1
                     0	    -0.08	 0; %(rx, ry, rz) link 2
                    -0.0475	 0       0;  %(rx, ry, rz) link 3
                     0       0       0.2085;%(rx, ry, rz) link 4
                     0       0       0;%(rx, ry, rz) link 5
                     0       0       -0.054];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    .13     .524	.539	0	0	0;
    .066	.086	.0125	0	0	0;
    1.8e-3	1.3e-3	1.8e-3	0	0	0;
    .3e-3	.4e-3	.3e-3	0	0	0;
    .15e-3	.15e-3	.04e-3	0	0	0];



robot.motors=load_motors([6 7 6 8 9 6]);
%Speed reductor at each joint
robot.motors.G=[30 325 180 75 1 1];
%q=[0 pi/2 pi 0 0 0];
