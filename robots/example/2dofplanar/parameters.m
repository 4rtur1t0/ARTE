%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   example 2 DOF planar robot.
%
%   Author: Arturo Gil. Universidad Miguel Hern�ndez de Elche. 
%   email: arturo.gil@umh.es date:   03/03/2012
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

robot.name='Example 2DOF planar arm';

%kinematic data DH parameters
robot.DH.theta='[q(1) q(2)]';
robot.DH.d='[0  0]';
robot.DH.a='[1  1]';
robot.DH.alpha='[0  0]';

%number of degrees of freedom
robot.DOF = 2;

%rotational: R, translational: T
robot.kind=['R' 'R'];

%Jacobian matrix. It is easy to obtain the Jacobian matrix of this robot
% J is defined such that:
% v=[vn wn]', where vn is the linear speed of the end effector and wn in
% the angular speed. vn=[vx vy vz] and wn = [wx wy wz]. Being this a planar
% robot, vz=0 and wx=wy=0
robot.J=['[-a(1)*sin(q(1))-a(2)*sin(q(1)+q(2))  -a(2)*sin(q(1)+q(2));' ... 
          'a(1)*cos(q(1))+a(2)*cos(q(1)+q(2))   a(2)*cos(q(1)+q(2));' ...
          '               0                                  0;' ...
          '               0                                  0;' ...
          '               0                                  0;' ...
          '               1                                  1]'];
%robot.J=[];

%Function name to compute inverse kinematic
robot.inversekinematic_fn = 'inversekinematic_2dofplanar(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                deg2rad(-180) deg2rad(180)]; %Axis 2, minimum, maximum
                
%maximum absolute speed of each joint rad/s or m/s
robot.velmax = []; %empty, not available

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

% end effectors maximum velocity
robot.linear_velmax = 0; %m/s, example, not available


%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GRAPHICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%read graphics files
robot.graphical.has_graphics=1;
robot.graphical.color = [25 20 40];
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-2.2 2.2 -2.2 2.2 0 2.2]
robot = read_graphics(robot);


%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot.q=[0 0]';
robot.qd=[0 0]';
robot.qdd=[0 0]';
robot.time = [];

robot.q_vector=[];
robot.qd_vector=[];
robot.qdd_vector=[];


robot.last_target=directkinematic(robot, robot.q);
robot.last_zone_data = 'fine';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction or not
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[1 1];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[-0.5      0         0; %(rx, ry, rz) link 1, w/r to reference system 1
                      -0.5      0         0];%(rx, ry, rz) link 2

%link masses
m1 = robot.dynamics.masses(1);
m2 = robot.dynamics.masses(2);

%eval a to obtain parameters.
a=eval(robot.DH.a);
L1 = a(1);
L2 = a(2);


%Inertia matrices of each link
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, or each row
robot.dynamics.Inertia=[0   m1*L1^2/3   m1*L1^2/3    0	0	0;
                        0   m2*L2^2/3   m2*L2^2/3    0	0	0];


%Inertia of the rotor
robot.motors.Inertia=[0 0];
%Reduction ration motor/joint speed
robot.motors.G=[1  1];


%Viscous friction factor of the motor
robot.motors.Viscous = [0  0];
%Coulomb friction of the motor
%Tc+, Tc-
robot.motors.Coulomb = [0	0;
            0	0];
        
%Obtained from motor catalog under practicals/inverse_dynamics
%                        R(Ohm)  L(H)      Kv (V/rad/s):speed constant     Kp (Nm/A):torque constant        Max_current (A) 
robot.motors.constants=[0.345  0.273e-3       2.3474e-05               84.9e-3                 139];%these correspond to Maxon, 167132;
  

        
        

