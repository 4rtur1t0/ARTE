%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB1600.
%
%   Author: . FALO Universidad Miguel Hernández de Elche. 
%   email: FALO@umh.es date:   09/01/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by FALO
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

robot.name= 'abb_irb1600_X145_m2004';

%Path where everything is stored for this robot
%robot.path = 'robots/abb/irb1600_X145_m2004';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.4865 0 0 0.6 0 0.065]';
robot.DH.a='[0.15 0.7 0 0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_abb_irb1600_X145_m2004(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                deg2rad(-90) deg2rad(150); %Axis 2
                deg2rad(-245) deg2rad(65); %Axis 3: Unlimited (400º default)
                deg2rad(-200) deg2rad(200); %Axis 4
                deg2rad(-115) deg2rad(115);  %Axis 5: Unlimited (800º default)
                deg2rad(-400) deg2rad(400)]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(150); %Axis 1, rad/s
                deg2rad(160); %Axis 2, rad/s
                deg2rad(170); %Axis 3, rad/s
                deg2rad(320); %Axis 4, rad/s
                deg2rad(400); %Axis 5, rad/s
                deg2rad(460)];%Axis 6, rad/s

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
            % end effectors maximum velocity
robot.linear_velmax = 2.5; %m/s

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);

% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 20 51]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1.75 1.75 -1.75 1.75 0 2];
%read graphics files
robot = read_graphics(robot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[0 157.5 70 20 2.25 0.25]; %Total 250kg

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0.1          0; %(rx, ry, rz) link 1
    -0.4	 0	 0; %(rx, ry, rz) link 2
    0	0	 0.3;  %(rx, ry, rz) link 3
    0       0       0;%(rx, ry, rz) link 4
    0       0           -0.075;%(rx, ry, rz) link 5
    0       0         -0.0032];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    1.3     5.24	5.39	0	0	0;
    1	1.2	0.3	0	0	0;
    0.12	0.1	0.12	0	0	0;
    .3e-2	.4e-2	.3e-2	0	0	0;
    .15e-2	.15e-2	.04e-2	0	0	0];
robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];