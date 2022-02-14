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

robot = load_robot('UR', 'UR5');
% adjust 3D view as desired
%adjust_view(robot)

delta_time = 50/1000;

q0 = pi/8*[1 1 1 1 1 1]';
q1 = pi/4*[1 1 1 1 1 1]';
drawrobot3d(robot, q0)
T_target = directkinematic(robot, q1)

q = pi/8*[1 1 1 1 1 1]';
% qs = []
% vref = [1 0 0];
% for i=1:10
%    J = manipulator_jacobian(robot, q);
%    Jv = J(1:3,:)
%    iJ = pinv(Jv)
%    qd = iJ*vref'
%    %eye = pinv(Jv)*Jv
%    q = q + qd*delta_time
%    qs = [qs q]
% end
% figure, plot(qs', 'Linewidth', 5)
        
qinv = inversekinematic(robot, T_target, q0)

T_reached = directkinematic(robot, qinv)
'diff T-T_target'
T_reached-T_target
% % Plot manipulatiliby ellipse
% drawrobot3d(robot, qinv)
