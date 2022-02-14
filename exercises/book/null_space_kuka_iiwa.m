% A SYMBOLIC DERIVATION OF THE JACOBIAN MANIPULATOR OF A KUKA LBR IIWA 14
%
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
function null_space_kuka_iiwa
null_space_7DOF()
null_space_6DOF()

null_space_6DOF_m3()

function null_space_7DOF()
% link lengths
robot = load_robot('KUKA', 'LBR_IIWA_R820_COP')
q = [pi/8 pi/8 pi/8 pi/8 pi/8  pi/8 pi/8 ];

J = manipulator_jacobian(robot, q)
iJm = moore_penrose(J)

% null space
ns = (eye(7)-iJm*J)*[1 0 0 0 0 0 0]'

[U,S,V] = svd(J)

function null_space_6DOF()
% link lengths
robot = load_robot('UR', 'UR5')
q = [pi/8 pi/8 pi/8 pi/8 pi/8 pi/8];

J = manipulator_jacobian(robot, q)

iJm = moore_penrose(J)

% null space
ns = (eye(6)-iJm*J)*[1 0 0 0 0 0]'

[U,S,V] = svd(J)

function null_space_6DOF_m3()
% link lengths
robot = load_robot('UR', 'UR5')
q = [pi/8 pi/8 pi/8 pi/8 pi/8 pi/8];

J = manipulator_jacobian(robot, q)

J = J(1:3, :);

iJm = moore_penrose(J)

% null space
ns = (eye(6)-iJm*J)*[1 0 0 0 0 0]'

[U,S,V] = svd(J)


function iJm = moore_penrose(J)
iJm = J'*inv(J*J');

        
