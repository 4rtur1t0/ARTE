% SCRIPT TEST THE DIRECT DYNAMICS OF A 2 DOF PLANAR ROBOT ROBOT

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

fprintf('\nTHE SIMULATION PRESENTS THE ROBOT AT AN INITIAL POSITION WHEN NO TORQUES ARE APPLIED\n')

%load robot parameters
robot=load_robot('example', '2dofplanar');

%simulate for 10 seconds, change this depending on your computer speed and
%the total time that you want to simulate
total_simulation_time = 10; 

%initial position and joint speed
q0 = [0 0]';
qd0 = [0 0]';

drawrobot3d(robot, q0);
adjust_view(robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The student should try different combinations of the following parameters:
%   g: the direction of the gravity vector. In this case, if we select g=[0  0 9.81]'; 
%       the movement of the arm is not affected by the gravity, since it
%       moves in a plane perpendicular to the gravity vector.
%   tau: the torques applied to each joint. The student should observe the effects of selecting
%       tau = [0 0 0]' or different values in combination with the
%       direction of the vector g.
%   robot.dynamics.friction = 0 selects no friction at the joints, whereas
%       robot.dynamics.friction = 1 considers that there exists friction. This
%       friction is modelled by robot.motors.Viscous (viscous friction) and
%       robot.motors.Coulomb (Coulomb friction). The student should observe
%       that selecting g=[0  9.81 0]' and tau = [0 0 0]' and
%       robot.dynamics.friction = 0 turns into an infinite triple pendulum
%       movement. In addition, selecting selecting g=[0  9.81 0]' and tau = [0 0 0]' and
%       robot.dynamics.friction = 1 simulates the case in which the triple
%       pendulum converges to a steady solution with the three links
%       hanging along the Y direction.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%you may redefine the gravity vector
%in this case you may one of the next two lines, that define
%the gravity acting along the Y axis or the Z axis, respectively.
g=[0  -9.81 0]'; %y0 axis


tau = [0 0]';%no torques applied
%Next, try this. Compute the torques needed to reach a particular motion
%state, such as q=[0 0], qd=[0 0] and qdd=[0 0]. That is, the arm is fully
%deployed. Note that, if we apply these torques, the acceleration computed
%by the inverse dynamic function should be zero, thus, the arm should not
%move.
%tau = inversedynamic(robot, [0 0], [0 0], [0 0], [0 -9.81 0]' ,[0 0 0 0 0 0]);

%select friction or not
robot.dynamics.friction = 0;

fprintf('\nCOMPUTING FORWARD DYNAMICS (this may take a while)')

%this may take a while, since it requires integration
%of the acceleration at each time step
[t q qd] = forwarddynamic(robot, total_simulation_time, q0, qd0, tau, g, []);

%animate it!!
animate(robot, q)

figure, plot(t, q), grid, title('Position vs. time')
xlabel('time (s)'), ylabel('Position (rad)')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');

figure, plot(t, qd), grid, title('Speed vs. time')
xlabel('time (s)'), ylabel('Speed (rad/s)')
legend('qd_1', 'qd_2', 'qd_3', 'qd_4', 'qd_5', 'qd_6');