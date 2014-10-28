%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   RX160L.
%
%   Author: RUBEN HERNANDEZ SANCHEZ
%                     ADRIAN DELGADO GARCIA
%                     ALEJANDRO MONTOYA GRACIA
%                     CARMEN MIRALLES PEREZ. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   09/01/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  
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

robot.name= 'RX160L';

%Path where everything is stored for this robot
robot.path = 'robots/staubli/RX160L';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3)+pi/2 q(4) q(5) q(6)]';
robot.DH.d='[0.55 0 0 0.925 0 0.11]';
robot.DH.a='[0.15 0.825 0 0 0 0]';
robot.DH.alpha= '[-pi/2 0 pi/2 -pi/2 pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_rx160l(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-160) deg2rad(160); %Axis 1, minimum, maximum
                deg2rad(-137.5) deg2rad(137.5); %Axis 2, minimum, maximum
                deg2rad(-150) deg2rad(150); %Axis 3
                deg2rad(-270) deg2rad(270); %Axis 4: Unlimited (400º default)
                deg2rad(-105) deg2rad(120); %Axis 5
                deg2rad(-270) deg2rad(270)]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(200); %Axis 1, rad/s
                deg2rad(200); %Axis 2, rad/s
                deg2rad(255); %Axis 3, rad/s
                deg2rad(315); %Axis 4, rad/s
                deg2rad(360); %Axis 5, rad/s
                deg2rad(870)];%Axis 6, rad/s
            
            robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);

% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [250 20 40]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1.5 1.5 -1.5 1.5 0 3.2];
%read graphics files
robot = read_graphics(robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%link masses (kg)
robot.dynamics.masses=[62.594 70.748 27.260 38.302 0.756 0.053];

%consider friction in the computations
robot.dynamics.friction=0;


%COM of each link with respect to own reference system
robot.dynamics.r_com=[-0.13 0.47   -0.18; %(rx, ry, rz) link 1
     0       0         0; %(rx, ry, rz) link 2
    0.05	-0.55	 -0.14;  %(rx, ry, rz) link 3
    0       0.004       0;%(rx, ry, rz) link 4
    0.01       0           0;%(rx, ry, rz) link 5
    0       0         0];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[1.158 -0.031 -0.224 1.42 -0.038 1.469;
    7.277 0 0 7.641 0.011 0.671;
    0.344 0.007 -0.007 0.292 -0.001 0.329;
    1.909 0 0.059 1.893 0 0.237;
    0.001 0 0 0.001 0 0.001;
    0 0 0 0 0 0];

robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];