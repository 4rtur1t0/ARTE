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
function speed_demo_scara
global robot
% joint position
q = [pi/4 pi/4 0.2 0]';
% joint speed
qd = [1 1 1 1]';
%robot = load_robot

J = manipulator_jacobian(robot, q);

Ve=J*qd;

%plot speed
T = directkinematic(robot, q);
p0 = T(1:3,4);
drawrobot3d(robot, q)
draw_vector(Ve(1:3), p0, 'linear speed V', 2)
draw_vector(Ve(4:6), p0, 'angular speed W', 1)


