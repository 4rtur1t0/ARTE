%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q = inversekinematic_5R(robot, T)
% Inverse kinematics for the 5R planar parallel robot
% The 5R robot is divided into two different 2DOF arms.
% INVERSEKINEMATIC_5R(robot, T)
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions for q = [Q1 Q2 Q3 Q4] 
% that place the end effectors at the position specified by T. 
% Any Qi is a 4 value vector that contains: Qi={q1 fi1, q2, fi2},
%
% where q1, fi1, are the joint variables of the first arm
%  and  q2, fi2, the joint variables of the second arm
%
% Four possible solutions q = [Q1 Q2 Q3 Q4],
% generally called elbow up and elbow down in four different combinations
% for the two different arms.
%
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 08/03/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function Q = inversekinematic_5R(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
P=T(1:3,4);

%transformation between the reference systems corresponding to the first
%and second arm
T2=robot.robot2.T0;
%the same point expressed in the second reference frame
P2=inv(T2)*[P; 1];
T2(1:3,4)=P2(1:3);

%solve the two first variables using inversekinematic for the first arm
q1=inversekinematic(robot.robot1, T);

%solve the two second variables using inversekinematic for the second arm
q2=inversekinematic(robot.robot2, T2);


Q=[q1(1,1) q1(1,2) q1(1,1) q1(1,2);
   q1(2,1) q1(2,2) q1(2,1) q1(2,2);
   q2(1,1) q2(1,1) q2(1,2) q2(1,2);
   q2(2,1) q2(2,1) q2(2,2) q2(2,2)];

