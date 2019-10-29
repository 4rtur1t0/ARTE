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
function speed_demo_partial_jacobian
robot = load_robot('example', 'scara')

% DIRECT JACOBIAN
%joint position
q = [0 0 0 0]';
%joint speed
qd = [1 1 1]';

a = eval(robot.DH.a);
J=eval(robot.J);
v1 = J*[1 1 1]'

%joint position
q = [pi/2 0 0 0]';
%joint speed
qd = [1 1 1]';

a = eval(robot.DH.a);
J=eval(robot.J);
v2 = J*[1 1 1]'


% INVERSE JACOBIAN

q = [pi/2 -pi/2 0 0]';
a = eval(robot.DH.a);
J=eval(robot.J);
%joint speed
v = [1 1 1]';
qd = inv(J)*v

