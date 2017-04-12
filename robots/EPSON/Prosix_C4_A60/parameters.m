%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   Epson Prosix_C4_A60.
%
%   The author of this script is:
%   Vicente Cabanes Ribelles
%   date:25/01/2017
%   Máster Robótica
%   Universidad Miguel Hernández Elche.
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

robot.name= 'Epson_Prosix_C4_A60';

%Prosix C4 A60
%Path where everything is stored for this robot
robot.path = 'robots/Epson/Prosix_C4_A60';

%Prosix C4 A60
robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)]';
robot.DH.d='[0.320 0 0 0.250 0 0.065]';
robot.DH.a='[0.100 0.250 0 0 0 0]';
robot.DH.alpha='[-pi/2 pi pi/2 -pi/2 pi/2 0]';
robot.J=[];

robot.inversekinematic_fn = 'inversekinematic_epson_c4_a60(robot, T)';

%Prosix C4 A60
%number of degrees of freedom
robot.DOF = 6;

%Prosix C4 A60
%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];


%Prosix C4 A60
%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-160) deg2rad(65); %Axis 2, minimum, maximum
                deg2rad(-51) deg2rad(225); %Axis 3
                deg2rad(-200) deg2rad(200); %Axis 4: 
                deg2rad(-135) deg2rad(135); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6: 
            
%Prosix C4 A60
%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(450); %Axis 1, rad/s
                deg2rad(450); %Axis 2, rad/s
                deg2rad(514); %Axis 3, rad/s
                deg2rad(555); %Axis 4, rad/s
                deg2rad(555); %Axis 5, rad/s
                deg2rad(720)];%Axis 6, rad/s
            
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet

%base reference system 
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
%read graphics files
robot.graphical.has_graphics=1;
%Color verdecito
robot.graphical.color = [225 20 40]./255;;
%for transparency
robot.graphical.draw_transparent=1;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.5 0.5 -0.5 0.5 0 1];
%read graphics files
robot = read_graphics(robot);


%DYNAMICS PARAMETERS
robot.has_dynamics=1;
%consider friction in the computations
robot.dynamics.friction=0;

%Prosix C4 A60
%link masses (kg)
robot.dynamics.masses=[0 18.9 6.3 1.07 0.59 0.14];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 1
                    -0.3638	 0.006	 0.2275; %(rx, ry, rz) link 2
                     -0.0203	-0.0141	 0.070;  %(rx, ry, rz) link 3
                     0       0.019       0;%(rx, ry, rz) link 4
                     0       0           0;%(rx, ry, rz) link 5
                    0         0         0.0320];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
                        0.13    0.524	0.539	0	0	0;
                        0.066	0.086	0.0125	0	0	0;
                        0.0018	0.0013	0.0018	0	0	0;
                        3e-4	4e-4	3e-4	0	0	0;
                        1.5e-4	1.5e-4	4e-5	0	0	0];


%Please note that we are simulating the motors as presented in MAXON
%catalog
robot.motors=load_motors([5 5 5 5 5 5]);


%Actuator rotor inertia
%robot.motors.Inertia=[200e-6 200e-6 200e-6 33e-6 33e-6 33e-6];
%Speed reductor at each joint
%robot.motors.G=[-62.6111 107.815 -53.7063 76.0364 71.923 76.686];
%Please note that, for simplicity in control, we consider that the gear
%ratios are all positive
robot.motors.G=[62.6111 107.815 53.7063 76.0364 71.923 76.686];