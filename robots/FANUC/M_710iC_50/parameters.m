%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   FANUC MATE M710iC/50, FANUC Robotics Europe.
%
%   Authors: Carlos Carrazoni Perez, Juan Jose Perez Hernandez & David
%   Martinez Pascual
%   Universidad Miguel Hernandez de Elche.  
%   date:   7/1/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Copyright (C) 2019, by  Carlos Carrazoni Perez, Juan Jose Perez Hernandez
% & David Martinez Pascual
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


robot.name= 'Fanuc_M_710iC_50';

%Path where everything is stored for this robot
robot.path = 'robots/fanuc/M_710iC_50';
%Tabla de D-H
robot.DH.theta= '[q(1)+pi/2  q(2)+pi/2  q(3)  q(4)  q(5)  q(6)]';
robot.DH.d='[0.565 0 0 1.016 0 0.175]';
robot.DH.a='[0.15 0.87 0.17 0 0 0]';
robot.DH.alpha= '[pi/2  0  pi/2  pi/2  -pi/2  0]';
robot.J=[];

robot.inversekinematic_fn = 'inversekinematic_fanuc_m710(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                deg2rad(-135) deg2rad(90); %Axis 2, minimum, maximum
                deg2rad(-160) deg2rad(280); %Axis 3
                deg2rad(-360) deg2rad(360); %Axis 4
                deg2rad(-125) deg2rad(125); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(175); %Axis 1, rad/s
                deg2rad(175); %Axis 2, rad/s
                deg2rad(175); %Axis 3, rad/s
                deg2rad(250); %Axis 4, rad/s
                deg2rad(250); %Axis 5, rad/s
                deg2rad(355)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, not specified
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.tool_activated=1;

% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [0.9 0.9 0]%./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1.5 1.5 -1.5 1.5 0 2];
%read graphics files
robot = read_graphics(robot);

%robot.tool=load_robot('equipment/end_tools','water_cutter');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[191.865 71.802 111.5 27.55 5.705 0.674];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[-0.109  -0.099  0.053; %(rx, ry, rz) link 1
                      -0.466  -0.001 -0.287; %(rx, ry, rz) link 2
                      -0.086   0.032  0.007;  %(rx, ry, rz) link 3
                       0       0.342  0.011;  %(rx, ry, rz) link 4
                       0      -0.015  0.076 ; %(rx, ry, rz) link 5
                       0       0      0.010];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[8.411      9.434	9.499   -2.810	0.691	1.396;
                        6.297     28.162	22.396  -0.049	-0.012	-9.398;
                        2.313      3.367	2.986	0.467	0.232	-0.285;
                        4.863	0.072	4.846	-0.002	0	0;
                        0.065	0.058	0.022	0	0.004   0;
                        0.000768	0.000767	0.001368	0	0	0];

%Speed reductor at each joint
robot.motors.G=[10 20 120 200 300 1];
