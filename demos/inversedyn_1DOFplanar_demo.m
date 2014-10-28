% SCRIPT TEST FOR THE 2DOF arm

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

q1 = [0 ];	
q2 = [pi/2];



%general call to inverse dynamic function
%TAU = inversedynamic(param, Q, QD, QDD, GRAV, FEXT)

%load example arm
planar=load_robot('example', '1dofplanar');


%Tau is cero at any joint at the presented position.
fprintf('\nTorques at each joint given position = [0 0], and velocity=[0 0] and standard gravity acting on Z0')
tau = inversedynamic(planar, q1, [0], [1], [0 0 9.81]', [0 0 0 0 0 0]')

drawrobot3d(planar,q1)
disp('press any key to continue')
pause


%now, change the direction of gravity, which is equivalent to change the
%mounting position of the arm. All torques appear as a consequence of a
%gravitational load acting on the COM of each link
fprintf('\nTorques at each joint given position = [0 0], and velocity=[0 0] and standard gravity acting on Y0')
tau = inversedynamic(planar, q1, [0 0], [0 0], [ 0 -9.81 0]', [0 0 0 0 0 0]')
disp('press any key to continue')
pause


%Now, apply forces expressed at the X2 Y2 Z2 reference system.
%Please note that forces acting on X2, Z2 and moments acting on My2 and Mx2
%do not contribute to the torques at each joint. Please add forces in Y2
%direction and moments acting on Z2 direction
fprintf('\nTorques at each joint given position = [0 0], and velocity=[0 0] and standard gravity acting on Z0, forces expressed on X2Y2Z2 [1 0 1 1 1 0]')
tau = inversedynamic(planar, q1, [0 0], [ 0 0], [0 0 9.81]', [1 0 1 1 1 0]')
disp('press any key to continue')
pause



%Now, compute torques necessary to bring the arm to a general kinetic state
fprintf('\nTorques at each joint given position = [0 0], and velocity=[0 0] and standard gravity acting on Z0, forces expressed on X2Y2Z2 [0 1 0 0 0 1]')
tau = inversedynamic(planar, q2, [1 1], [1 1], [0 0 9.81]', [0 1 0 0 0 1]')

drawrobot3d(planar,q2)
disp('press any key to continue')
pause


