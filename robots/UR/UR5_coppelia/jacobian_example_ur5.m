% SCRIPT TEST FOR THE UR5 ROBOT  JACOBIAN

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
function jacobian_example_ur5()


close all
robot = load_robot('UR', 'UR5_coppelia');
% qué direcciones de vx vy vz wx wy wz pueden ser realizadas por el robot?
% se trata de un punto singular?
% q0 = [0, 0, 0,  0, 0.0, 0.0];
% drawrobot3d(robot, q0)
% J = manipulator_jacobian(robot, q0)
% J
% det(J)

%
q0 = [0, 0, -pi/2,  0, pi/2, 0.0];
drawrobot3d(robot, q0)
J = manipulator_jacobian(robot, q0)
Jv = J(1:3, :)
Jw = J(4:6, :)

det(Jv*Jv')
null(Jv)

det(Jw*Jw')
null(Jw)


