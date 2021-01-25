% SCRIPT TEST to view a manipulability ellipse

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
fprintf('\nView manipulability ellipse and forces ellipse at q')

robot = load_robot('UR', 'UR10')

q = [pi/2 pi/4 -pi/4 0.4 0.5 -pi/4]';
%q = [0.05 0.05 0.05 0.05 0.05 0.05]';

%Plot manipulatiliby ellipse
drawrobot3d(robot, q)
hold on
%draw manipulability ellipse
plot_manipulability_ellipse(robot, q, 1)
%draw forces ellipse
plot_forces_ellipse(robot, q, 1)