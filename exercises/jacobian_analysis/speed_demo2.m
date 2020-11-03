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
function speed_demo2
close all
robot = load_robot('KUKA', 'KR6_2')

fprintf('\nPRESS ANY KEY TO CONTINUE...')
pause

%joint position
q = [0.5 0.3 0.0 0 -pi/2 0]'
%joint speed
qd = [0.3 0.3 0.3 0.3 0.3 0.3]'


for i=1:30,
    T = directkinematic(robot, q)
    J = manipulator_jacobian(robot, q);
    V = compute_end_velocity(robot, q, qd)

    %plot speed
    p0 = T(1:3,4)
    drawrobot3d(robot, q)
    draw_vector(V(1:3), p0, 'linear speed V', 3)
    draw_vector(V(4:6), p0, 'angular speed W', 2)
    q = q + qd*0.1;
    pause(0.1)
end

