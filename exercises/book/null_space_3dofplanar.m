% The null space of a 3DOF robot when considered redundant.
% if the task m=(vx, vy), then the robot is redundant in lambda=3
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
function null_space_3dofplanar
% link lengths
robot = load_robot('example', '3dofplanar');
q = [pi/2 pi/2 pi/2]';

qs = [];
qds = [];
manips = [];
for i=1:300
    [qb, manip] = null_movement(robot, q);
    q = q + 0.1*qb;
    qs = [qs q];
    qds = [qds qb];
    manips = [manips manip];
end

animate(robot, qs(:,1:15:end))
figure, plot(manips), legend('manipulability')
figure, plot(qs'), legend('q_1', 'q_2', 'q_3')
figure, plot(qds'), legend('qd_1', 'qd_2', 'qd_3')
%q = q + 0.01*ns';
    

function [ns, manip] = null_movement(robot, q)
J = manipulator_jacobian(robot, q);
% consider only vx, vy
J = J(1:2, :);
iJm = moore_penrose(J);
P = (eye(3)-iJm*J)
% project to null space
ns = P*[1 0 0]';
manip = det(J*J')


function iJm = moore_penrose(J)
iJm = J'*inv(J*J');

        
