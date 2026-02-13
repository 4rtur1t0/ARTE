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


robot.name= 'RRR';

robot.DH.theta= '[q(1)   q(2)   q(3) ]';
robot.DH.d=     '[0.3     0      0   ]';
robot.DH.a=     '[0.0    0.5    0.5]';
robot.DH.alpha= '[pi/2    0      0  ]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_RRR(robot, T)';

%number of degrees of freedom
robot.DOF = 3;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                 deg2rad(-90) deg2rad(270);   %Axis 2, minimum, maximum
                 deg2rad(-90) deg2rad(+90)]; %Axis 3:

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(136); %Axis 1, rad/s
                deg2rad(95); %Axis 2, rad/s
                deg2rad(120)];%Axis 6, rad/s
            
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
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[10 10 10]; % m1, m2, m3

%COM of each link with respect to own reference system [m]
robot.dynamics.r_com=[0	      -0.15	       0;  %(rx, ry, rz) link 1
                     -0.25	     0	       0;  %(rx, ry, rz) link 2
                     -0.25	     0	       0]; %(rx, ry, rz) link 3

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row [kg*m^2]
% moments of inertia with respect each of the center of 
% masses
%robot.dynamics.Inertia=[0.02146      0.02146    0.00125 0	0	0;
%                        0.02167	     0.02167    0.00167 0	0	0;
%                        0.02167	     0.02167    0.00167 0	0	0];
robot.dynamics.Inertia=[1.0      1.0    1.0  0	0	0;
                        1.0	     1.0    1.0  0	0	0;
                        1.0	     1.0    1.0  0	0	0];

                   

robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];

