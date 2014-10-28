%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  JOINT_REFERENCES
%  Obtain q when the robot makes a linear trajectory of the end effector 
%  in cartesian space
%
%  See also INVERSEKINEMATIC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function q=joint_references(time)

global robot

time;
if time == 0
    robot.q=[0 0 0 0 0 0]';
end

simulation_time = 0.5;%s

%this is important, as this will be the initial joint
%values in the first integrator
%initial position
T1 = directkinematic(robot, [0 0 0 0 0.1 0]);
%final position
T2 = directkinematic(robot, [0.3 0.3 0.3 0.1 0.1 0.1]);

Q1 = T2quaternion(T1);
Q2 = T2quaternion(T2);
%interpolate between the two orientations
[Qm] = slerp(Q1, Q2, time/simulation_time, 0.01);



initial_point=T1(1:3,4);
end_point=T2(1:3,4);

% %NOA matrix end point
% T2=[1 0 0 0.5;
%     0 1 0 0.4;
%     0 0 1 0.3; 
%     0 0 0  1];

%speed of the movement
speed = norm(initial_point-end_point)/simulation_time;%m/s

v=(end_point-initial_point);
v=v/norm(v); %unit vector in the direction of the line

point = initial_point + speed*time*v;

%T1(1:3,4)=point(1:3);

Ttotal = quaternion2T(Qm, point(1:3));
qinv = inversekinematic(robot, Ttotal);
q = select_closest_joint_coordinates(robot, qinv, robot.q);

%update current robot joints
robot.q=q;
