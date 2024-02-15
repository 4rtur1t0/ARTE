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
function test_kinematics_ur5()
%test_pm_in_circle();%
%test_q1_valid();

close all
robot = load_robot('UR', 'UR5_coppelia');
%q0 = [pi/4 -pi/2.5 -pi/2.5 -3*pi/2 pi/2 pi/2]';
%q0 = [pi/8, -pi/3, -pi/3,  3*pi/4, pi / 2, pi / 2]
%q0 = [0, -pi/2, pi/2,  0, 0.0, 0.1]
q0 = [0, 0, 0,  pi, 0.0, 0.0]
drawrobot3d(robot, q0)
T = directkinematic(robot, q0);
qinv = inversekinematic(robot, T);

fprintf('FOUND SOLUTIONS')
qinv

for i=1:size(qinv, 2)
    T2 = directkinematic(robot, qinv(:,i));
    drawrobot3d(robot, qinv(:,i))
    pause(1);
    E = (T2-T)
    E = (T2-T).^2;
    E = sum(sum(E));
    if E > 0.01
        error('ERROR en cinematica')
    end
end

fprintf('FOUND SOLUTIONS')
qinv

end

function test_wrist()
    close all
    robot = load_robot('UR', 'UR5_coppelia');
    q0 = [pi/4 pi/4 pi/4 pi/4 pi/4 pi/4]';
    drawrobot3d(robot, q0)
    T = directkinematic(robot, q0);
    %qinv = inversekinematic(robot, T);
    A01 = dh(robot, q0, 1);
    A12 = dh(robot, q0, 2);
    A23 = dh(robot, q0, 3);
    A34 = dh(robot, q0, 4);
    A45 = dh(robot, q0, 5);
    A03 = A01*A12*A23;
    A04 = A01*A12*A23*A34;
    A05 = A01*A12*A23*A34*A45;
    z3 = A03(1:3, 3);
    z4 = A04(1:3, 3);
    z5 = A05(1:3, 3);
    z4T = A04(1:3,4);
end



function test_q1_valid()

close all
robot = load_robot('UR', 'UR5_coppelia');
q0 = [pi/4 -pi/4 -pi/4 pi/8 pi/8 pi/8]';
drawrobot3d(robot, q0)
T = directkinematic(robot, q0);
qinv = inversekinematic(robot, T);

end


function test_pm_in_circle()
close all

robot = load_robot('UR', 'UR5_coppelia');
% adjust 3D view as desired
%adjust_view(robot)

%q0 = [0 0 0 0 0 0]';
q0 = pi/8*[1 1 1 1 1 1]';
drawrobot3d(robot, q0)
T = directkinematic(robot, q0)
%qinv = inversekinematic(robot, T)

%  test that when changing q4 q5 q6, pm is at a circle
p = [];
for i=1:500
    q = q0 + [0 0 0 normrnd(0,2) normrnd(0,2) normrnd(0,2)]';
    pm = find_pm(robot, q);
    p = [p pm];
end
figure,
plot(p(1, :), p(3, :), 'b.')
plot3(p(1,:), p(2,:), p(3,:), 'b.')
end

function pm=find_pm(robot, q)
%eight possible solutions are generally feasible
%q=zeros(6,8);
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
% test position pm
pm = p - L6*z6;
pm = pm(:);
end






