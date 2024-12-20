% SCRIPT TEST FOR THE UR10 ROBOT KINEMATICS

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
close all
robot = load_robot('practicals', 'UR10');
adjust_view(robot)

% DESCOMENTE LA L�NEA SIGUIENTE PARA PROBAR UN PUNTO SINGULAR
q0 = [0.1 0.1 0.1 0.1 0.1 0.1]';
q = [0.1 -pi/2 pi/2 pi/4 pi/4 pi/4]';

T = directkinematic(robot, q);

fprintf('\nSimple test: try to reach T')
% Llame a la cinemática inversa 
qinv = inversekinematic(robot, T, q0)

T_reach = directkinematic(robot, qinv)
'diff T-Treach'
T-T_reach



