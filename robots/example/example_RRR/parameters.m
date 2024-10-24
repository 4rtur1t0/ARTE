%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   example RRR robot.
%
%   Author: Arturo Gil
%   email: arturo.gil@umh.es
%   date:   18/11/2024
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


robot.name= 'KR180_R2100';

robot.DH.theta= '[q(1)   q(2)-pi/2   q(3)+pi/2 ]';
robot.DH.d='[0.57   0   0   ]';
robot.DH.a='[0.3   0.9   0.5   ]';
robot.DH.alpha= '[-pi/2   0   0   ]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_kuka_kr180_r2100(robot, T)';

%number of degrees of freedom
robot.DOF = 3;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[(-165) (165); %Axis 1, minimum, maximum
                (-135) (45);   %Axis 2, minimum, maximum
                (-65) (155);  %Axis 3
                (-350) (350);  %Axis 4:
                (-125) (125);  %Axis 5
                (-350) (350)] *pi/180; %Axis 6:

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(136); %Axis 1, rad/s
                deg2rad(95); %Axis 2, rad/s
                deg2rad(120); %Axis 3, rad/s
                deg2rad(179); %Axis 4, rad/s
                deg2rad(172); %Axis 5, rad/s
                deg2rad(219)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, not specified

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [15 76 129]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-2 2 -2 2 0 2];
%read graphics files
robot = read_graphics(robot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=0;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[4 6 3 2.5 1 0.5];

%COM of each link with respect to own reference system [m]
robot.dynamics.r_com=[  0	      0	       0;    %(rx, ry, rz) link 1
                     -0.017	    0.005	  0.08;  %(rx, ry, rz) link 2
                     -0.019	   -0.012	  0.05;  %(rx, ry, rz) link 3
                        0       0.017      0;    %(rx, ry, rz) link 4
                        0         0        0;    %(rx, ry, rz) link 5
                        0         0       0.04]; %(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row [kg*m^2]
robot.dynamics.Inertia=[0.024     0.018	   0.019   	0	0	0;
                        0.063	  0.051	   0.022	0	0	0;
                        0.005	  0.009	   0.009	0	0	0;
                        0.005	  0.006    0.0082	0	0	0;
                        .8e-3	  .0012    .0013	0	0	0;
                        .17e-3    .09e-3   .09e-3	0	0	0];



robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];

