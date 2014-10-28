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
%robot=load_robot('example','5R');

%Now test the directkinematic function in speed.
% We simulate that the robot is moving from a joint position 
% qA=[pi/4 3*pi/4]=[q1 q2]; 
% qB=[pi/3 3pi/4+pi/3]
% at each time step the angular joint speeds qd1 and qd2 are pi/3 rad/s
q1=linspace(pi/4, pi/3, 50);
q2=linspace(3*pi/4, 3*pi/4+pi/5, 50);

vxs=[];
p0s=[];
figure, hold
for i=1:length(q1),
    %calling the specific 5R kinematic function !!
    [T, vx, ax, phi, phid]=directkinematic_5R(robot, [q1(i) q2(i) pi/3 pi/3]);   

    p0=T(1:3,4);
    vxs=[vxs vx(:,1)];
    p0s=[p0s p0];
    %drawrobot3d(robot, [q1(i) phi(1,1) q2(i) phi(2,1)]);
    %vect_arrow(p0, [x y z], 'r', 3);
       
    x = p0(1) + 0.5*vx(1)';
    y = p0(2) + 0.5*vx(2)';
    z = 0;

    vect_arrow(p0, [x y z]', 'r', 3);
end

% plot the speed at each of the points
% origins p0
[T, vx, ax, phi, phid]=directkinematic_5R(robot, [q1(1) q2(1) pi/3 pi/3]);   

drawrobot3d(robot, [q1(1) phi(1,1) q2(1) phi(2,1)]); 
figure, hold
for i=1:length(q1),
    p0=p0s(:,i);
    vx=[vxs(:,i); 0];
    x = p0(1) + 0.5*vx(1)';
    y = p0(2) + 0.5*vx(2)';
    z = 0;

    vect_arrow(p0, [x y z]', 'r', 3);
end



