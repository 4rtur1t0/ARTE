%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB1600iD.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Authors:Daniel Vivancos Unica
%        Jose David Martinez Exposito
%        Maria Jose Martinez Liza
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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

robot.name= 'ABB_IRB1600ID';

%Path where everything is stored for this robot
robot.path = 'robots/abb/IRB1600ID';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.4865 0 0 0.640 0 0.2]';
robot.DH.a='[0.15 0.7 0.11 0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_irb1600id(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[-pi pi; %Axis 1, minimum, maximum
                deg2rad(-90) deg2rad(150); %Axis 2, minimum, maximum
                deg2rad(-238) deg2rad(79); %Axis 3
                deg2rad(-155) deg2rad(155); %Axis 4
                deg2rad(-90) deg2rad(135); %Axis 5
                deg2rad(-200) deg2rad(200)]; %Axis 6

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(180); %Axis 1, rad/s
                deg2rad(180); %Axis 2, rad/s
                deg2rad(180); %Axis 3, rad/s
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
robot.axis=[-2 2 -2 2 0 1.2];
%read graphics files
robot = read_graphics(robot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%DYNAMICS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
%Tenemos que repartir 250kg (peso total robot IRB1600ID entre los 6 eslabones)    
%Utilizando como material Aluminio, los datos obtenidos del programa Inventor seran:

robot.dynamics.masses=[0 109.440 28.242 31.079 6.936 1.430 0.249]
%COM of each link with respect to own reference system
robot.dynamics.r_com=[0.052       -0.012    0.343; %(rx, ry, rz) link 1
         0.150       0.791     -0.183;%(rx, ry, rz) link 2
         1.172	     0.140	   0.017; %(rx, ry, rz) link 3
         1.296       0.063     0.561; %(rx, ry, rz) link 4
         1.296       0.870     0.027; %(rx, ry, rz) link 5
         0           1.296     0.966];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0	    0   	0	0	0;
         2.305  2.696	3.802	0	0	0;
         .0700	1.765	1.784	0	0	0;
         0.267  0.324	0.332	0	0	0;
         0.297	0.290	.023	0	0	0;
         .008	.002	.008	0   0	0;
         0      0       0       0   0   0];
%Los momentos de Inercia y los centros de gravedad quedan corregidos de los
%obtenidos del programa Inventor para ser ajustados al sistema de
%coordenadas designado en nuestro robot.


robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];
