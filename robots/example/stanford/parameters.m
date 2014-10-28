%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   Stanford robot.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   10/04/2012
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
%KYNEMATICS
robot.name= 'stanford';

robot.DH.theta = '[q(1)   q(2)  -pi/2  q(4)   q(5)   q(6)]';
robot.DH.d=     '[0.412  0.154   q(3)          0      0    0.263]';
robot.DH.a=     '[  0     0      0          0      0     0]';
robot.DH.alpha= '[-pi/2  pi/2    0         -pi/2  pi/2   0]';
robot.J=[];
robot.name='Stanford robotic arm';


robot.inversekinematic_fn = 'inversekinematic_stanford(robot, T)';

%R: rotational, T: translational
robot.kind=['R' 'R' 'T' 'R' 'R' 'R'];

%number of degrees of freedom
robot.DOF = 6;

%minimum and maximum rotation angle in rad
%Please note that these values are approximate
robot.maxangle =[deg2rad(-160) deg2rad(160); %Axis 1, minimum, maximum
                deg2rad(-110) deg2rad(110); %Axis 2, minimum, maximum
                -0.5 0.5; %Axis 3, translational, m
                deg2rad(-266) deg2rad(266); %Axis 4: Unlimited (400º default)
                deg2rad(-100) deg2rad(100); %Axis 5
                deg2rad(-266) deg2rad(266)]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [1
                1
                1
                1
                1
                1];%not available
            
            robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 0.5; %m/s, from datasheet


%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GRAPHICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%read graphics files
robot.graphical.has_graphics=0;
robot.graphical.color = [25 20 40];
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1 1 -1 1 0 1];
robot = read_graphics(robot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DYNAMIC PARAMETERS
% As extracted from:
%- Kinematic data from "Modelling, Trajectory calculation and Servoing of 
%   a computer controlled arm".  Stanford AIM-177.  Figure 2.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[9.29 5.01  4.25 1.08 0.630 0.51];


%COM of each link with respect to own reference system
robot.dynamics.r_com=[0    0.0175 -0.1105; %(rx, ry, rz) link 1
    0   -1.054  0; %(rx, ry, rz) link 2
   0    0      -6.447;  %(rx, ry, rz) link 3
   0    0.092  -0.054;%(rx, ry, rz) link 4
   0    0.566   0.003;%(rx, ry, rz) link 5
    0    0       1.554];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0.276   0.255   0.071   0   0   0;
        0.108   0.018   0.100   0   0   0;
         2.51    2.51    0.006   0   0   0 ;
      0.002   0.001   0.001   0   0   0 ;
      0.003   0.0004  0       0   0   0;
      0.013   0.013   0.0003  0   0   0];

robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];
