%
% SCRIPT TO TEST THE KINEMATICS OF THE 3RRR robot
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
%the robot is represented (interiorly) by three planar 2dof arms
robot=load_robot('example','3RRR');

adjust_view(robot)


%init T
T=eye(4);
%Represent a final point in space that will be achieved by the end effector
%P=[xA, yA]=[1.5 1.5]
T(1,4)=1.0;
T(2,4)=1.0;
%orientation
phi=pi/3;
T(1,1)=cos(phi)
T(2,1)=sin(phi)

%find 8 solutions for the INVERSE KINEMATIC in position
q=inversekinematic(robot, T)

%plot every solution
drawrobot3d(robot, q(:,1)), pause(2);
drawrobot3d(robot, q(:,2)), pause(2);
drawrobot3d(robot, q(:,3)), pause(2);
drawrobot3d(robot, q(:,4)), pause(2);
drawrobot3d(robot, q(:,5)), pause(2);
drawrobot3d(robot, q(:,6)), pause(2);
drawrobot3d(robot, q(:,7)), pause(2);
drawrobot3d(robot, q(:,8)), pause(2);


%Make a line in space with a variation in the angle phi
x=linspace(1,1.4,50);
y=linspace(1.1,1.15,50);
th=linspace(0,pi/3,50);
Q=[];
for i=1:length(x),
    T(1,4)=x(i);
    T(2,4)=y(i);
    %orientation

    T(1,1)=cos(th(i));
    T(2,1)=sin(th(i))
    
    q=inversekinematic(robot, T);
    
    
    drawrobot3d(robot, q(:,1), 1) %noclear activated
    drawrobot3d(robot, q(:,1)) %noclear deactivated
end


%TEST NOW the directkinematic function 
T(1,4)=1.0;
T(2,4)=1.0;
%orientation
phi=pi/3;
T(1,1)=cos(phi)
T(2,1)=sin(phi)

%find 8 solutions for the INVERSE KINEMATIC in position
q=inversekinematic(robot, T)

%now, for the first solution
T=directkinematic(robot, [q(1,1) q(3,1) q(5,1)])
%caution: there may exist less than 8 possible solutions
for i=1:8, 
    drawrobot3d(robot, q(:,i))
    % 1 and 3 correspond to the joint variables of the robot
    T=directkinematic(robot, [q(1,i) q(3,i) q(5,i)])
    pause(2);
    close all
end

