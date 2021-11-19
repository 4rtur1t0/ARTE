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
fprintf('\nView forces ellipse at q')
%CAution: the q = 0 position produces an error when trying to find the 
% main directions and eigen values.
q = [0.0 0.0 0.0 0.0 0.01 0.0]';
J = manipulator_jacobian(robot, q);
f = [0 0 1 0 0 0]';
tau = J'*f;

% %Plot manipulatiliby ellipse
drawrobot3d(robot, q)
hold on
%draw forces ellipse
plot_forces_ellipse(robot, q, 0.00003)
