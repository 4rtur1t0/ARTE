%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB140.
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
%   email: arturo.gil@umh.es date:   09/01/2012
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

robot.name= 'motoman_MH5LF';

robot.DH.theta= '[q(1) q(2)+pi/2 q(3) q(4) q(5) q(6)+pi/2]';
robot.DH.d='[0.33 0 0 0.405 0 0.08]';
robot.DH.a='[0.088 0.4 0.04 0 0 0]';
robot.DH.alpha= '[pi/2 0 pi/2 -pi/2 pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_MH5LF(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[-pi pi; %Axis 1, minimum, maximum
                deg2rad(-100) deg2rad(100); %Axis 2, minimum, maximum
                deg2rad(-220) deg2rad(60); %Axis 3
                deg2rad(-200) deg2rad(200); %Axis 4: Unlimited (400� default)
                deg2rad(-120) deg2rad(120); %Axis 5
                deg2rad(-400) deg2rad(400)]; %Axis 6: Really Unlimited to (800� default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(250); %Axis 1, rad/s
                deg2rad(150); %Axis 2, rad/s
                deg2rad(190); %Axis 3, rad/s
                deg2rad(300); %Axis 4, rad/s
                deg2rad(300); %Axis 5, rad/s
                deg2rad(420)];%Axis 6, rad/s
    
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 2.5; %m/s



%base reference system
robot.T0 = eye(4);


%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.5 0.75 -0.75 0.75 0 1.1];
%read graphics files
robot = read_graphics(robot);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC PARAMETERS
%   WARNING! These parameters do not correspond to the actual IRB 140
%   robot. They have been introduced to demonstrate the necessity of 
%   simulating the robot and should be used only for educational purposes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[4.0850 7.64 2.991 8.3833 0.1558 0.0298];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[-0.06      -0.099          0; %(rx, ry, rz) link 1
                    -0.20779	    0	         0; %(rx, ry, rz) link 2
                   -0.20779	        0 	        -5.5;  %(rx, ry, rz) link 3
                     0             0.147        0;%(rx, ry, rz) link 4
                     0              0           0.038;%(rx, ry, rz) link 5
                     0              0         -0.0135];%(rx, ry, rz) link 6
%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0.0588      0.0440	0.0772   	-0.0243	0	0;
    0.0353       0.5310	      0.5070        0   0   0;
    0.0166	     0.0110	      0.0185        0 	0	0.36898e-5;
    0.2967	     0.0418	      0.2967	    0	0	0;
    0.33502e-5	0.33502e-5	  0.70128e-6	0	0	0;
    0.0052	    0.0052	      0.72020e-6	0	0	0];


robot.motors=load_motors([4 5 4 5 5 5]);%no hemos podido introducir  
%estos motores en el fichero load_motor.m puesto que en el catalogo de los motores abb
%faltaban datos   R(Ohm)  L(H)    Kv       Nominal_speed
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];

