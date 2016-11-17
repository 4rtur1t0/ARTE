%
% SCRIPT TO TEST THE KINEMATICS OF THE 5R robot
%
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

%load the robot
%the robot is represented (interiorly) by two planar 2dof arms
robot=load_robot('example','5R');

adjust_view(robot)


%Represent a final point in space that will be achieved by the end effector
%P=[Px, Py]=[0.6 0.4]
T=eye(4);
T(1,4)=1.0;%change, for example, to 1.25
%T(2,4)=0.967; %change, for example to 0.967
T(2,4)=0.8; %change, for example to 0.967

%find 4 solutions for the INVERSE KINEMATIC in position
q=inversekinematic(robot, T)

drawrobot3d(robot, q(:,1)), pause(1);
drawrobot3d(robot, q(:,2)), pause(1);
drawrobot3d(robot, q(:,3)), pause(1);
drawrobot3d(robot, q(:,4)), pause(1);
close all
%now solve the DIRECTKINEMATIC problem for each of the above computed
% q
for i=1:4,
    drawrobot3d(robot, q(:,i))
    figure, hold
    % 1 and 3 correspond to the joint variables of the robot
    T=directkinematic(robot, [q(1,i) q(3,i)])
    pause(2);
    close all
end

