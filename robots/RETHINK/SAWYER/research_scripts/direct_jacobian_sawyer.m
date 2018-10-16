%
% Just a demo to test the Jacobian on the Sawyer robot.
%
% DIRECT JACOBIAN DEMO

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
% along with ARTE.  If not, see <http://www.gnu.odrg/licenses/>.

close all;

fprintf('\nThe demo shows how to compute the end effectors speed as a function of the joint speeds')

robot =  load_robot('RETHINK','SAWYER');

T=1; %seconds. Tiempo que dura el movimiento

qi=0:0.01:pi/2;

q_v=1; %rad/s
V=zeros(6, length(qi));
for i=1:length(qi),
   v = compute_end_velocity(robot, [qi(i) qi(i)  qi(i)  qi(i) qi(i) qi(i) qi(i)], [q_v q_v q_v q_v q_v q_v q_v]);
   V(:,i) = v(1:6);
end

figure, hold
plot(V(1, :), 'r')
plot(V(2, :), 'g')
plot(V(3, :), 'b')
legend('V_x', 'V_y', 'V_z')
title('End effector linear speed (m/s)')
xlabel('time (s)')

figure, hold
plot(V(4, :), 'r')
plot(V(5, :), 'g')
plot(V(6, :), 'b')
legend('w_x', 'w_y', 'w_z')
title('End effector angular speed (rad/s)')
xlabel('time (s)')

figure, hold
plot(qi, 'r')
legend('q_i')
title('Joint positions (rad)')
xlabel('time (s)')
