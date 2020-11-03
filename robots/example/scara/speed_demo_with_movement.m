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
function speed_demo_with_movement
close all
robot = load_robot('example', 'scara')

% DIRECT JACOBIAN
%joint position
q = [0 0 0 0]';
%joint speed
qd = [pi/2 pi/2 0.5 0]';

qs = [];
vs = [];
for i=1:30
    % caution: a must be defined in order to eval(robot.J)
    a = eval(robot.DH.a);
    J = eval(robot.J);
    % J = manipulator_jacobian(robot, q)
    v = J*qd(1:3);
    q = q + qd*0.1;
    qs = [qs q];
    vs = [vs v];
end

figure, plot(qs(1,:),'rs'), hold
plot(qs(2,:),'g*')
plot(qs(3,:),'bd')
legend('q1 (rad)', 'q2 (rad)', 'q3 (m)')

figure, plot(vs(1,:),'rs'), hold
plot(vs(2,:),'g*')
plot(vs(3,:),'bd')
legend('vx m/s', 'vy m/s', 'vz m/s')

