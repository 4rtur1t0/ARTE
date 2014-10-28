%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   KUKA KR5 scara Z200.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   05/01/2012
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

robot.DH.theta= '[ q(1) q(2)    0      q(4)]';
robot.DH.d='[     0.246  0      -q(3)+0.246 0]';
robot.DH.a='[     0.125  0.225   0     0]';
robot.DH.alpha= '[0      pi       pi     0]';
robot.J=[];
robot.name= 'KUKA_KR5_scara_R350_Z200';

robot.inversekinematic_fn = 'inversekinematic_KUKA_KR5_scara_R350_Z200(robot, T)';

%number of degrees of freedom
robot.DOF = 4;

%rotational: R, translational: T
robot.kind=['R' 'R' 'T' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-155) deg2rad(155); %Axis 1, minimum, maximum
                deg2rad(-145) deg2rad(145); %Axis 2, minimum, maximum
                0.046                0.200; %Axis 3, translational, max 200 mm
                deg2rad(-358) deg2rad(358)]; %Axis 4
                
%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(525); %Axis 1, rad/s
                deg2rad(525); %Axis 2, rad/s
                deg2rad(2); %Axis 3, m/s
                deg2rad(2400)]; %Axis 4, rad/s
               
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, not specified

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

% GRAPHICS
%read graphics files
robot.graphical.has_graphics=1;
robot.graphical.color = [255 20 40]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.75 0.75 -0.75 0.75 0 0.8];
robot = read_graphics(robot);

%DYNAMICS
%NOTE THAT THIS DYNAMIC PARAMETERS ARE NOT REAL,
%JUST USED FOR TEACHING PURPOSES HERE
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=1;

%link masses (kg)
robot.dynamics.masses=[15 10 5 1] ;

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 1
                     -0.05	 0.006	 0.1; %(rx, ry, rz) link 2
                    -0.0203	-0.0141	 0.070;  %(rx, ry, rz) link 3
                     0       0.019       0 ];%(rx, ry, rz) link 4

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    .13     .524	.539	0	0	0;
    .066	.086	.0125	0	0	0;
    1.8e-3	1.3e-3	1.8e-3	0	0	0];



robot.motors=load_motors([5 5 5 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300];

