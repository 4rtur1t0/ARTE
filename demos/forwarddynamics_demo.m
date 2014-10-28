% SCRIPT TEST THE DIRECT DYNAMICS OF THE PUMA 560 ROBOT

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
% You should have received a copy of the GNU Lesser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.

fprintf('\nTHE SIMULATION PRESENTS THE ROBOT AT AN INITIAL POSITION WHEN NO TORQUES ARE APPLIED\n')

%load robot parameters
robot=load_robot('unimate', 'puma560');

total_simulation_time = 2; %simulate for 1 second

%initial position and joint speed
q0 = [0 0 0 0 0 0]';
qd0 = [0 0 0 0 0 0]';

drawrobot3d(robot, q0);
adjust_view(robot);

%try both
tau = [0 0 0 0 0 0]';%no torques applied
%tau = [0 200 1 1 1 1]';

%no friction
robot.friction = 0;

fprintf('\nCOMPUTING FORWARD DYNAMICS (this may take a while)')

%this may take a while, since it requires integration
%of the acceleration at each time step
[t q qd] = forwarddynamic(robot, total_simulation_time, q0, qd0, tau, []);

%animate it!!
animate(robot, q)

figure, plot(t, q), grid, title('Position vs. time')
xlabel('time (s)'), ylabel('Position (rad)')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');

figure, plot(t, qd), grid, title('Speed vs. time')
xlabel('time (s)'), ylabel('Speed (rad/s)')
legend('qd_1', 'qd_2', 'qd_3', 'qd_4', 'qd_5', 'qd_6');