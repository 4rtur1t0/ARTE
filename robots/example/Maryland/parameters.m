%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of a
%   DELTA robot with 3 degrees of freedom
%
%   Author: Ángel Rodríguez 
%   email: arodgre@gmail.com date:   18/12/2013
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

path=pwd;

%For this special case, the mechanism is formed by three different
%3DOF spherical arms.

robot1=load_robot('example','3dofsphericalb');
robot2=load_robot('example','3dofsphericalb');
robot3=load_robot('example','3dofsphericalb');

robot=[];
robot.name='Example MARYLAND 3DOF (X Y Z)';
%treated separately in the plotting tools
robot.parallel=1;

robot.robot1=robot1;
robot.robot2=robot2;
robot.robot3=robot3;

%number of kinematic chains
robot.nserial=3;


%Kinematics
robot.inversekinematic_fn='inversekinematic_Maryland3DOF(robot,T)';
robot.directkinematic_fn='directkinematic_Maryland3DOF(robot,q)';
%robot.drawrobot3D_fn='drawDELTA3DOF(robot,T)';
robot.DOF=9;

robot.debug=1;
robot.T0=eye(4);


%Geometric Parameters (m, rad) or each arm
%d1 is the separation of the base's triangle
robot.rb=0.5;
%d2 is the separation of the effector's triangle
robot.re=0.25;
%L1 is the length of the upper arms
robot.L1=0.3;
%L2 is the length of the lower arms
robot.L2=0.7;
%Phi is an array of the angles between X axis and the arms
robot.Phi=[0, 2*pi/3, 4*pi/3];%[0º,120º,240º]
%robot.Phi=[pi/3, pi, 5*pi/3];

%dynamic parameters of each arm
robot.mp=2; %kg, mass of the end effector
robot.mb=0.5; %kg, mass of each of the two beams that form the upper arm
robot.ma=1; %mass of the low arms


% initial position for (0,0,-0.45)
%the whole joint vector position is defined as:
% q is q=[th1 phi11 phi12 th2 phi21 phi22 th3 phi31 phi32 ]
% being th1, th2 th3 active coordinates and phi{ii} passive
robot.q=[0.0548 2.4189 0 0.0548 2.4189 0 0.0548 2.4189 0];


%please note that the transformation between any of the system can be
%represented by a DH transformation with theta=Phi_i, d=0, a=rb, alpha=pi/2
robot.robot1.T0=dh(robot.Phi(1), 0, robot.rb, pi/2);
robot.robot2.T0=dh(robot.Phi(2), 0, robot.rb, pi/2);
robot.robot3.T0=dh(robot.Phi(3), 0, robot.rb, pi/2);



%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
%robot=init_sim_variables(robot);
robot.path = path;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GRAPHICS   Author: Arturo Gil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%read graphics files
robot.graphical.has_graphics=1;
robot.graphical.color = [255 20 40]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-3.5 3.5 -3.5 3.5 0 1]
robot = read_graphics(robot);

% robot.equipment=load_robot('example/Delta/base_plate');
% robot.tool=load_robot('example/Delta/effector');