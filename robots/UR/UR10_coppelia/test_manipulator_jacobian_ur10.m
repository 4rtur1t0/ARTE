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

robot = load_robot('UR', 'UR10_coppelia');

% q = [0 0 0 0 0 0]'
% J = manipulator_jacobian(robot, q)
% 
% q = [1 1 1 1 1 1]'
% J = manipulator_jacobian(robot, q)
% qd =inv(J)*[1 0 0 0 0 0]'
% 
% 
% q = [.1 .1 .1 .1 .1 .1]'
% J = manipulator_jacobian(robot, q)
% qd = inv(J)*[1 1 1 1 1 1]'

delta_time = 50/1000;
q = [pi/4 pi/4 pi/4 pi/4 pi/4 pi/4]'
vref = [0.1 0.1 0.1 0.1 0.1 0.1];
qs = []
ps = []
for i=1:10
    T = directkinematic(robot, q)
    J = manipulator_jacobian(robot, q);
    qd = inv(J)*vref';
    q = q + qd*delta_time
    qs = [qs q];
    ps = [ps T(1:3, 4)]
    
end
close all
figure, 
plot(qs')
close all
figure,
animate(robot, qs)

