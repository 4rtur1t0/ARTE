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
robot=load_robot('example','Delta');

adjust_view(robot)


%Represent a final point in space that will be achieved by the end effector
%P=[Px, Py, Pz]=[0.6 0.4 -0.2]

T=eye(4);
T(1,4)=-0.2;%change, for example, to 0.3
T(2,4)=0.3; %change, for example to 0.6
T(3,4)=-0.5;

%find 8 solutions for the INVERSE KINEMATIC in position
q=inversekinematic(robot, T);

drawrobot3d(robot, q(:,8)), pause(2);
drawrobot3d(robot, q(:,7)), pause(2);
drawrobot3d(robot, q(:,6)), pause(2);
drawrobot3d(robot, q(:,5)), pause(2);
drawrobot3d(robot, q(:,4)), pause(2);
drawrobot3d(robot, q(:,3)), pause(2);
drawrobot3d(robot, q(:,2)), pause(2);
drawrobot3d(robot, q(:,1)), pause(2);
 

% now solve the DIRECTKINEMATIC problem for each of the above computed q
% In this case, only the "down" solution is returned
T=directkinematic(robot,q(:,1))
T=directkinematic(robot,q(:,2))
T=directkinematic(robot,q(:,3))
T=directkinematic(robot,q(:,4))
T=directkinematic(robot,q(:,5))
T=directkinematic(robot,q(:,6))
T=directkinematic(robot,q(:,7))
T=directkinematic(robot,q(:,8))

