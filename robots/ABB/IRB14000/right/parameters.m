%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   IRB YUMI robot from ABB.
%
% Authors: Marc Fabregat y Antonio Martinez
% Universidad Miguel Hernández de Elche
% Enero de 2020
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
robot.name= 'IRB14000_right';


robot.DH.theta= '[q(1)       q(2)            q(3)            q(4)-pi/2      q(5)+pi       q(6)+pi             q(7)]';
robot.DH.d='[    0.166        0               0.2515           0     0.265       0        0.036]';
robot.DH.a='[    -0.03       0.03                0.0405             0.0405        0.027        0.027           0.0]';
robot.DH.alpha= '[-pi/2        pi/2               -pi/2          -pi/2      -pi/2         -pi/2         0]';
robot.J=[];


robot.inversekinematic_fn = 'inverse_kinematics_jacobian_moore(robot, T, q)';
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 7;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[
                deg2rad(-168.5) deg2rad(168.5); %Axis 2, minimum, maximum
                deg2rad(-143.5) deg2rad(43.5); %Axis 3
                deg2rad(-168.5) deg2rad(168.5); %Axis 4: Unlimited (400??? default)
                deg2rad(-123.5) deg2rad(80); %Axis 5
                deg2rad(-290) deg2rad(290);
                deg2rad(-88) deg2rad(138); 
                deg2rad(-229) deg2rad(229)]; %Axis 7: Really Unlimited to (800??? default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(180); %Axis 1, rad/s
                deg2rad(180); %Axis 2, rad/s
                deg2rad(180); %Axis 3, rad/s
                deg2rad(180); %Axis 4, rad/s
                deg2rad(400); %Axis 5, rad/s
                deg2rad(400);
                deg2rad(400)];%Axis 7, rad/s
    
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 2.5; %m/s



%base reference system
robot.T0 =  [0.5768    0.0708    0.8138   -0.0058;
    0.3769    0.8608   -0.3420   -0.0476;
   -0.7247    0.5040    0.4698    0.3807;
         0         0         0    1.0000];



%this is the coupling transformation
%between the last reference system and the piece
robot.Tcoupling=[-1 0 0  0;
     0 1 0  0;
     0 0 -1 0;
     0 0 0 1];


%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [93 188 210]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=0;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1.2 1.2 -1.2 1.2 0 1.3];
%read graphics files
robot = read_graphics(robot);


%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC PARAMETERS
%   WARNING! These parameters do not correspond to the actual IRB 140
%   robot. They have been introduced to demonstrate the necessity of 
%   simulating the robot and should be used only for educational purposes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=0;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[30.984 0.655 0.966 0.492 0.809 0.235 0.322 0.029];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 1
                     -0.05	 0.006	 0.1; %(rx, ry, rz) link 2
                    -0.0203	-0.0141	 0.070;  %(rx, ry, rz) link 3
                     0       0.019       0;%(rx, ry, rz) link 4
                     0       0           0;%(rx, ry, rz) link 5
                     0       0         0.032];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    .13     .524	.539	0	0	0;
    .066	.086	.0125	0	0	0;
    1.8e-3	1.3e-3	1.8e-3	0	0	0;
    .3e-3	.4e-3	.3e-3	0	0	0;
    .15e-3	.15e-3	.04e-3	0	0	0];



robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];
%}
%SPECIAL PARAMETERS TO SOLVE THE INVERSE KINEMATICS
robot.parameters.step_time=0.1;
%Error in XYZ to stop inverse kinematics
robot.parameters.epsilonXYZ=0.001;
%Error in Quaternion to stop inverse kinematics.
robot.parameters.epsilonQ=0.001;
robot.parameters.stop_iterations=500;

