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

%For this special case, the mecanisme is divided into three different siral
%arms wich are 3DOF spherical arms.

robot1=load_robot('example','3dofspherical');
robot2=load_robot('example','3dofspherical');
robot3=load_robot('example','3dofspherical');

robot=[];
robot.name='Example DELTA 3DOF';
%treated separately in the plotting tools
robot.parallel=1;

robot.robot1=robot1;
robot.robot2=robot2;
robot.robot3=robot3;
robot.nserial=3;


%Kinematics
robot.inversekinematic_fn='inversekinematic_DELTA3DOF(robot,T)';
robot.directkinematic_fn='directkinematic_DELTA3DOF(robot,q)';
%robot.drawrobot3D_fn='drawDELTA3DOF(robot,T)';
robot.DOF=9;

robot.debug=1;
robot.T0=eye(4);




%Geometrical Parámeters (m, rad)

%d1 is the radius of the base's triangle
robot.d1=0.5;
%d2 is the radius of the efector's triangle
robot.d2=0.25;
%L1 is the length of the upper arms
robot.L1=0.3;
%L2 is the length of the lower arms
robot.L2=0.7;
%Alpha is an array of the angle between X axis and the arms
robot.alpha=[pi/3, pi, 5*pi/3];%[60º,180º,40º]
% initial position for (0,0,-0.45)
robot.q=[0.0548 2.4189 0 0.0548 2.4189 0 0.0548 2.4189 0];

alpha=robot.alpha;
d1=robot.d1;
d2=robot.d2;

robot.TX=[1     0           0     0;
          0 cos(-pi/2)  -sin(-pi/2) 0;
          0 sin(-pi/2)   cos(-pi/2) 0;
          0     0          0      1  ];%Turn over X axis to put Z as the rotational axes of each joint

TX=robot.TX;

T01_1= [cos(alpha(1)) -sin(alpha(1)) 0 d1*cos(alpha(1));
        sin(alpha(1)) cos(alpha(1))  0 d1*sin(alpha(1));
        0 0 1 0; 
        0 0 0 1];

T01_2= [cos(alpha(2)) -sin(alpha(2)) 0 d1*cos(alpha(2));
        sin(alpha(2)) cos(alpha(2)) 0 d1*sin(alpha(2));
        0 0 1 0; 
        0 0 0 1];

T01_3= [cos(alpha(3)) -sin(alpha(3)) 0 d1*cos(alpha(3)); 
        sin(alpha(3)) cos(alpha(3)) 0 d1*sin(alpha(3));
        0 0 1 0; 
        0 0 0 1];

robot.robot1.T0=T01_1*TX;
robot.robot2.T0=T01_2*TX;
robot.robot3.T0=T01_3*TX;

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
%robot=init_sim_variables(robot);
robot.path = path;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GRAPHICS   Autor: Arturo Gil
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

robot.equipment=load_robot('example/Delta/base_plate');
robot.tool=load_robot('example/Delta/effector');