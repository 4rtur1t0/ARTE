%
% SCRIPT TO TEST THE DYNAMICS OF THE PUMA 560 ROBOT
%
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
%TAU = inversedynamic(robot, Q, QD, QDD, GRAV, FEXT)

% uncomment the following lines to simulate the stanford arm
% other arms can be simulated similarly
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% three different poses, please find which one is the worst
% pose for the first joint, the second joint etc.
% q1 = [0 0 0 0 0 0];	
% q2 = [0 pi/2 0.5 0 0 0];	
% q3 = [0 0 -0.5 0 0 0];
% 
% %load robot parameters
% robot=load_robot('stanford', '');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% three different poses, please find which one is the worst
% pose for the first joint, the second joint etc.
q1 = [0 0 0 0 0 0];	
q2 = [0 pi/2 -pi/2 0 0 0];	
q3 = [0 0 -pi/2 0 0 0];


%load robot parameters
robot=load_robot('unimate', 'puma560');

adjust_view(robot)
robot.dynamics.friction=1;

%Now compute torques at each pose
fprintf('\nTorques at each joint given position q1, zero speed and acceleration, standard gravity acting on Z0')
fprintf('\nComputing static torques at position q1 due to gravity [0 0 9.81]')
tau = inversedynamic(robot, q1, [0 0 0 0 0 0], [0 0 0 0 0 0], [0  0 9.81]', [0 0 0 0 0 0]')
drawrobot3d(robot,q1)
disp('press any key to continue')
pause


fprintf('\nTorques at each joint given position q2, zero speed and acceleration, standard gravity acting on Z0')
fprintf('\nComputing static torques at position q2 due to gravity [0 0 9.81]');
fprintf('\nPLEASE note the differences with respect to torque tau_2 at q1');
tau = inversedynamic(robot, q2, [0 0 0 0 0 0], [0 0 0 0 0 0], [0  0 9.81]', [0 0 0 0 0 0]')
drawrobot3d(robot,q2)
disp('press any key to continue')
pause


fprintf('\nTorques at each joint given position q3, zero speed and acceleration, standard gravity acting on Z0')
fprintf('\nComputing static torques at position q3 due to gravity [0 0 9.81]');
fprintf('\nPLEASE note the differences with respect to torque tau_2 at q1');
tau = inversedynamic(robot, q3, [0 0 0 0 0 0], [0 0 0 0 0 0], [0  0 9.81]', [0 0 0 0 0 0]')
drawrobot3d(robot,q3)
disp('press any key to continue')
pause


fprintf('\nTorques at each joint given position q3, zero speed and acceleration, standard gravity acting on Z0')
fprintf('\nComputing static torques at position q3 due to a 5 Kg load at end effector');
fprintf('\nPLEASE note the differences with respect to torque tau_2 at q1');
M = 5 %Kg
tau = inversedynamic(robot, q3, [0 0 0 0 0 0], [0 0 0 0 0 0], [0  0 M*9.81]', [0 0 0 0 0 0]')
drawrobot3d(robot,q3)
disp('press any key to continue')
pause


%NOW TEST THE DYNAMICS WITH A FORCE APPLIED AT THE END EFFECTOR
fprintf('\nTorques at each joint given position q3, zero speed and acceleration, standard gravity acting on Z0')
fprintf('\nComputing static torques at position q3 due to gravity and 1 Newton applied at Y6');
fprintf('\nPLEASE compare the result with the previous case');
tau = inversedynamic(robot, q3, [0 0 0 0 0 0], [0 0 0 0 0 0], [0  0 9.81]', [0 1 0 0 0 0]')
disp('press any key to continue')
pause


%NOW TEST THE DYNAMICS WITH A GENERAL CASE
fprintf('\nTorques at each joint given position q3, general movement')
fprintf('\nPLEASE compare the result with the previous cases');
tau = inversedynamic(robot, q3, [1 1 1 1 1 1], [1 1 1 1 1 1], [0 0 9.81]', [1 1 1 1 1 1]')
disp('press any key to continue')
pause


%FINALLY, CONSIDER THE CASE IN WHICH THE MANIPULATOR IS HANGING FROM THE CEILING
fprintf('\nTorques at each joint given position q3, general movement')
fprintf('\nPLEASE compare the result with the previous cases');
tau = inversedynamic(robot, q3, [0 0 0 0 0 0], [0 0 0 0 0 0], [0  0 -9.81]', [0 0 0 0 0 0]')

%the robot is now hanging, Z0 pointing downwards
robot.T0=[-1 0 0 0;
          0 1 0 0;
          0 0 -1 1;
          0 0 0 1];

drawrobot3d(robot,q3)
disp('press any key to continue')
pause