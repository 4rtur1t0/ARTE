%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 	TEST THE DYNAMICS OF A 2DOF robot under different circumstances.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2016, by Arturo Gil Aparicio
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
function dynamics1()

%Just load the robot once
robot=load_robot('example', '2dofplanar');

%TODO: uncommment as you solve the exercises
%exercise1(robot)
exercise2(robot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Given the following specifications, find the maximum torques if:
%   
%   g = [0 -9.81 0]'
%   amax = [2 2] rad/s^2, max angular acceleration
%   wmax = [3 3], max angular speed
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise1(robot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TODO: CHANGE q to find the "worst" pose
q=[pi/2 pi/2]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qd=[2 2]'; %speed --> does it make any difference?

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: CHANGE qdd [-3 3] to find whether higher torques are achieved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qdd = [3 3]'; %--> max acceleration

g=[0 -9.81 0]'; % acting on the Y0 axis.

fext = [0 0 0 0 0 0]';

figure, drawrobot3d(robot, q)

%Compute the torques
tau1 = inversedynamic(robot, q, qd, qdd, g, fext)

%store the results and find the maximum torque

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Given the following specifications, find the maximum torques if:
%   
%   g = [0 -9.81 0]'
%   amax = [2 2] rad/s^2, max angular acceleration
%   wmax = [3 3], max angular speed
%   The robot carries a piece at the end effector with mass=2kg.
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise2(robot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TODO: CHANGE q to find the "worst" pose
q=[0 0]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qd=[2 2]'; %speed --> does it make any difference?

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: CHANGE qdd [-3 3] to find whether higher torques are achieved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qdd = [0 0]'; %--> max acceleration

m = 2; %kg

g=[0 -9.81 0]'; % acting on the Y0 axis.


fext = [ m*g'  0 0 0]';

figure, drawrobot3d(robot, q)

%Compute the torques
tau1 = inversedynamic(robot, q, qd, qdd, g, fext)

%store the results and find the maximum torque