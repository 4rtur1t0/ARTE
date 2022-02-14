% SIMPLE ALGORITHM TO FOLLOW A LINE IN SPACE. Error correction based on a P
% controller on the closest point to the line vector.
%
% Copyright (C) 2019, by Arturo Gil Aparicio
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
function weighed_jacobian


robot = load_robot('KUKA', 'LBR_IIWA_R820_7DOF')
q = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
M = diag([7 6 5 4 3 2 1]);
v = [1 1 1 1 1 1];
J = manipulator_jacobian(robot, q);
Jw = J*M;

mp1 = pinv(Jw)*v'

mp2 = (J*M)'*inv(J*M*(J*M)')*v'

mp3 = M*J'*inv(J*M*M*J')*v'


