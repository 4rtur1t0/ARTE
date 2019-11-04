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
function speed_demo
global robot
%joint position
q = [pi/4 pi/4 pi/4 0 -pi/2 0]'
%joint speed
qd = [1 1 1 0 0 5]'
%robot = load_robot

%J = compute_jacobian_exercise(robot, q);
J = manipulator_jacobian(robot, q);

Ve=J*qd;

qd = inv(J)*Ve;

V = compute_end_velocity(robot, q, qd)

%plot speed
T = directkinematic(robot, q)
p0 = T(1:3,4)
drawrobot3d(robot, q)
draw_vector(V(1:3), p0, 'linear speed V', 2)
draw_vector(V(4:6), p0, 'angular speed W', 1)

%find joint speeds
%caution! you may be at a singular point, modify q accordingly
qd=compute_joint_velocity(robot, q, V)
