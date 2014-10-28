% SCRIPT TEST FOR THE 3 DOF spherical manipulator

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

close all

%general call to inverse dynamic function
%TAU = inversedynamic(param, Q, QD, QDD, GRAV, FEXT)

%three different poses
q1 = [0, pi/2, pi/2];	
q2 = [0, pi/2, 0];


%load 3dof spherical robot
%the parameters are loaded from robots/example/3dofspherical/parameters.m
spherical=load_robot('example', '3dofspherical');

spherical.graphical.draw_transparent=1;

fprintf('\nTorques at each joint given position = [0 0 0], and velocity=[0 0 0] and standard gravity acting on Z0')
%Please note that the forces and moments are specified with respect to the
%X3 Y3 Z3 reference system.
%In this mechanism, forces acting on Fx3 or Fz3 do not account for tau, nor
%moments acting on My3 or Mx3
tau = inversedynamic(spherical, q1, [0 0 0], [0 0 0], [0 0 9.81]', [0 0 0 0 0 0]')

%draw the robot at this position
drawrobot3d(spherical,q1)
disp('press any key to continue')
pause

%Now, add a force on Fy, and observe the results on tau
tau = inversedynamic(spherical, q1, [0 0 0], [0 0 0], [0 0 9.81]', [0 1 0 0 0 0]')
%draw the robot at this position
drawrobot3d(spherical,q1)
disp('press any key to continue')
pause


%torques necessary to instantaneously bring the arm to the specified state
fprintf('\nTorques at each joint given position = [0 pi/2 0], and velocity=[1 1 1] and acceleration = [1 1 1] and gravity acting on Z0')
tau = inversedynamic(spherical, q2, [1 1 1], [1 1 1], [0 0 9.81]', [0 0 0 0 0 0]')

%draw the robot
drawrobot3d(spherical,q2)
disp('press any key to continue')
pause


%In this case, all torques are caused by gravity acting on the Center Of
%Mass of each link. Note that we have changed the direction of the gravity
%vector g
fprintf('\nTorques at each joint given position = [0 0 0], and velocity=[0 0 0] and acceleration = [0 0 0] and gravity acting on Y0')
tau = inversedynamic(spherical, q1, [0 0 0], [0 0 0], [0 9.81 0]', [0 0 0 0 0 0]')

%draw the robot
drawrobot3d(spherical,q1)


