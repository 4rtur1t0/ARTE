%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB1520_ID.
%
%   Author: Konrad Tolivia. Universidad Miguel Hernandez de Elche. 
%   email: konrad.tolivia@goumh.umh.es date:   09/05/2019
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

robot.name= 'ABB_IRB1520_ID';

robot.DH.theta= '[q(1) q(2)+pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.453 0 0 0.723 0 0.200]';
robot.DH.a='[0.160 0.590 0.200 0 0 0]';
robot.DH.alpha= '[pi/2 0 pi/2 -pi/2 pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_irb1520ID(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-90) deg2rad(150); %Axis 2, minimum, maximum
                deg2rad(-100) deg2rad(80); %Axis 3
                deg2rad(-155) deg2rad(155); %Axis 4: Unlimited (400� default)
                deg2rad(-90) deg2rad(135); %Axis 5
                deg2rad(-200) deg2rad(200)]; %Axis 6: Really Unlimited to (800� default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(130); %Axis 1, rad/s
                deg2rad(140); %Axis 2, rad/s
                deg2rad(140); %Axis 3, rad/s
                deg2rad(320); %Axis 4, rad/s
                deg2rad(380); %Axis 5, rad/s
                deg2rad(460)];%Axis 6, rad/s
    
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
robot.dynamics.masses=[37.37 56.35 23.21 5.61 1.1 0.192];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0      0.125; %(rx, ry, rz) link 1
                      0       0      0.25; %(rx, ry, rz) link 2
                      0.07    0      0.09;  %(rx, ry, rz) link 3
                      0       0      0.2;%(rx, ry, rz) link 4
                      0       0      0.1;%(rx, ry, rz) link 5
                      0       0      0.0175];%(rx, ry, rz) link 6
                 
%Para calcular las inercias respecto los sistemas DH, aproximamos a un 
%cilindro rigido, y aplicamos el teorema de Steiner. 
%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0.9919  0.9919  0.426   0	0	0;
    4.95     4.95	  0.5381 	0	0	0;
    0.367    0.367	  0.23  	0	0	0;
    0.3026	 0.3026   6.07e-3  	0	0	0;
    0.0149	 0.0149	  4.63e-4  	0	0	0;
    1.19e-4	 1.19e-4  8.07e-5  	0	0	0];



robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];

