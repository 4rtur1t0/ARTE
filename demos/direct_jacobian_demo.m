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

robot =  load_robot('example','scara');

T=1; %Tiempo que dura el movimiento

q1=0:0.01:pi/2;
q2=0:0.01:pi/2;

q_v=pi/2/T; %rad/s
V=zeros(3, length(q1));
for i=1:length(q1),
   V(:,i) = compute_end_velocity(robot, [q1(i) -q2(i) 0 0], [q_v q_v 0 0]);
end

figure, hold
plot(V(1, :), 'r')
plot(V(2, :), 'g')
plot(V(3, :), 'b')
legend('V_x', 'V_y', 'V_z')
title('End effector speed (m/s)')
xlabel('time (s)')

figure, hold
plot(q1, 'r')
plot(q2, 'g')
legend('q_1', 'q_2')
title('Joint positions (rad)')
xlabel('time (s)')
