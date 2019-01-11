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

robot.name= 'FANUC M_10iA';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.45 0 0 0.64 0 0.1 ]';
robot.DH.a='[0.15 0.6  0.2 0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_m10ia(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-340) deg2rad(340); %Axis 1, minimum, maximum
                deg2rad(-250) deg2rad(250); %Axis 2, minimum, maximum
                deg2rad(-447) deg2rad(447); %Axis 3
                deg2rad(-380) deg2rad(380); %Axis 4: Unlimited (400� default)
                deg2rad(-380) deg2rad(380); %Axis 5
                deg2rad(-720) deg2rad(720)]; %Axis 6: Really Unlimited to (800� default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(230); %Axis 1, rad/s
                deg2rad(225); %Axis 2, rad/s
                deg2rad(230); %Axis 3, rad/s
                deg2rad(430); %Axis 4, rad/s
                deg2rad(430); %Axis 5, rad/s
                deg2rad(630)];%Axis 6, rad/s
    
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 2.5; %m/s



%base reference system
robot.T0 = eye(4);


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
robot.graphical.draw_axes=0.7;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=0.7;
%adjust for a default view of the robot
robot.axis=[-0.5 1.2 -0.5 0.5 0 1.5];
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
robot.dynamics.masses=[45 40 25 15 4 1];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0.15       -0.05          0.05; %(rx, ry, rz) link 1
                     -0.25 0	 -0.15; %(rx, ry, rz) link 2
                        0.1 0 0.1;  %(rx, ry, rz) link 3
                     0       -0.3       -0.05;%(rx, ry, rz) link 4
                     0       0           0.02;%(rx, ry, rz) link 5
                     0       0         0.1];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    .13     .524	.539	0	0	0;
    .066	.086	.0125	0	0	0;
    0	0	0	0	0	0;
    0	0	0	0	0	0;
    0	0	0	0	0	0];



robot.motors=load_motors([4 4 4 7.5 7.5 11]);
%Speed reductor at each joint
robot.motors.G=[1200 715 700 10 10 1];

