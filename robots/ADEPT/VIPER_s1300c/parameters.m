%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   Adept Viper_s1300.
%
%   Author:  David Álvaro GarcÌa, Oscar Lillo Diaz, Mario Terres DÌaz
%   Universidad Miguel Hernández de Elche. 
%   email: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
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
% robot.DH.theta= '[q(1) q(2) q(3) q(4) q(5) q(6)+pi]';
%robot.DH.d='[0.475 0 0 0.59 0 0.09]';
%robot.DH.a='[0.180 0.520 -0.1 0 0 0]';
%robot.DH.alpha= '[-pi/2 0 pi/2 pi/2 0 -pi/2]';

% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function robot = parameters()

robot.name= 'Adept_Viper_s1300C';

%Path where everything is stored for this robot
robot.path = 'robots/adept/Viper_s1300';

robot.DH.theta= '[q(1) q(2) q(3) q(4) q(5) q(6)]';
robot.DH.d='[0.475 0 0 0.59 0 0.09]';
robot.DH.a='[0.180 0.520 -0.1 0 0 0]';
robot.DH.alpha= '[-pi/2 0 pi/2 pi/2 -pi/2 0]';

robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_viper_s1300(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-180) deg2rad(45); %Axis 2, minimum, maximum
                deg2rad(-10) deg2rad(255); %Axis 3, minimum, maximum
                deg2rad(-185) deg2rad(185); %Axis 4, minimum, maximum 
                deg2rad(-120) deg2rad(120); %Axis 5, minimum, maximum
                deg2rad(-360) deg2rad(360)]; %Axis 6, minimum, maximum 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(225); %Axis 1, rad/s
                deg2rad(169); %Axis 2, rad/s
                deg2rad(188); %Axis 3, rad/s
                deg2rad(375); %Axis 4, rad/s
                deg2rad(375); %Axis 5, rad/s
                deg2rad(600)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

%base reference system 
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [204 51 0]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-5 5 -5 5 0 5];
%read graphics files
robot = read_graphics(robot);


%DYNAMICS
robot.has_dynamics=1; 
%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[44.148 14.82 13.33 4.157 0.858 0.687];

%COM of each link with respect to own reference system (m)
robot.dynamics.r_com=[0.018153  -0.078706  0.148; %(rx, ry, rz) link 1
     -0.18  0.094  0.0041; %(rx, ry, rz) link 2
    -0.260  0.0029  0.0091;  %(rx, ry, rz) link 3
    -0.000878 0.000043 0.0823;%(rx, ry, rz) link 4
    -0.004461  -0.152 -0.000119;%(rx, ry, rz) link 5
     0.000349 -0.011501  0];%(rx, ry, rz) link 6

%Inertia matrix of each link with respect to its D-H reference system.
% Ixx  Iyy	Izz	Ixy	Iyz	Ixz, for each row (kg*m^2)
robot.dynamics.Inertia=[128.7  116.4  88  0	0	0;
    114.02  111.2  8.57   0	0	0;
    18.36	 14.15	  13.3   0 	 0	 0;
    15.2	 1.6	  15.3   0	 0	 0;
    0.20	 0.12	  0.17	   0	0	0;
    0.02	 0.04    0.02	   0	0	0];

robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];