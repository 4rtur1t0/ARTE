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
%
% Draws a circle in the robot workspace with the position
% of the point pw when changin q4, q5, q6
%

% SCRIPT TO DRAW A CIRCLE OF THE POINT PW (pseudo wrist in UR5)
function draw_circle_ur5_pw()

close all
robot = load_robot('UR', 'UR5');
q0 = [pi/4 -pi/6 -pi/6 pi/6 pi/6 pi/6]';

drawrobot3d(robot, q0)

%  test that when changing q4 q5 q6, pm is at a circle
p = [];
for i=1:2000
    q = q0 + [0 0 0 normrnd(0,2) normrnd(0,2) normrnd(0,2)]';
    pm = find_pm(robot, q);
    p = [p pm];
end
hold on,
plot3(p(1,:), p(2,:), p(3,:), 'b.', 'MarkerSize', 20)
end

function pm=find_pm(robot, q)
    T = directkinematic(robot, q);

    %Evaluate the parameters
    L6 = eval(robot.DH.d);
    %See geometry at the reference for this robot
    L6=L6(6);

    %Evaluate the parameters
    L4 = eval(robot.DH.d);
    %See geometry at the reference for this robot
    L4=L4(4);

    %action 1: position of pm (false wrist) when changing q6, q5, q4
    p = T(1:3, 4);
    z6 = T(1:3, 3);
    % compute position pm
    pm = p - L6*z6;
    pm = pm(:);
end






