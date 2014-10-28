%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the 3RRR planar parallel robot
% The 3RRR robot is divided into three different 2DOF arms.
%
% INVERSEKINEMATIC_3RRR(robot, T)
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions for q = [Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8] 
% that place the end effectors at the position specified by T. 
% Any Qi is a 4 value vector that contains: Qi={q1 fi1, q2, fi2, q3, fi3},
% being q1, fi1, the joint variables of the first arm
%  and  q2, fi2, the joint variables of the second arm
%  and  q3, fi3, the joint variables of the second arm
%
% Eight possible solutions q = [Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8],
% generally called elbow up and elbow down in eight different combinations for
% each of the three arms, as described in the following table:
%  
%               Arm 1       Arm 2      Arm 3
%   Sol 1        0           0          0   
%   Sol 2        0           0          1
%   Sol 3        0           1          0
%   Sol 4        0           1          1
%   Sol 5        1           0          0
%   Sol 6        1           0          1
%   Sol 7        1           1          0
%   Sol 8        1           1          1
%
%       1: elbow up solution
%       0: elbow down solution
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
function Q = inversekinematic_3RRR(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];


%Now find the orientation from the angle phi encoded in T=(ux, uy, uz)
u=T(1:3,1);
phi = atan2(u(2), u(1));

h=robot.h;

%Position of the point A in base coordinates
xA=T(1,4);
yA=T(2,4);

%Position of the point B in base coordinates
xB=xA+h*cos(phi);
yB=yA+h*sin(phi);

%Position of the point C in base coordinates
xC=xA+h*cos(phi+pi/3);
yC=yA+h*sin(phi+pi/3);


%Transformation of the second arm. Compute the point (xB, yB) in
%coordinates of the second arm
T2=robot.robot2.T0;
%the same point expressed in the second reference frame
P=inv(T2)*[xB; yB; 0; 1];
%store it in T2
T2(1:3,4)=P(1:3);


%transformation between the reference systems corresponding to the first
%and second arm
T3=robot.robot3.T0;
%the same point expressed in the second reference frame
P=inv(T3)*[xC; yC; 0; 1];
%Store it in T3
T3(1:3,4)=P(1:3);


%solve the two first variables using inversekinematic for the first arm
q1=inversekinematic(robot.robot1, T);

%solve the two second variables using inversekinematic for the second arm
q2=inversekinematic(robot.robot2, T2);

%solve the two first variables using inversekinematic for the first arm
q3=inversekinematic(robot.robot3, T3);


Q=[q1(1,1) q1(1,2) q1(1,1) q1(1,2) q1(1,1) q1(1,2) q1(1,1) q1(1,2);
   q1(2,1) q1(2,2) q1(2,1) q1(2,2) q1(2,1) q1(2,2) q1(2,1) q1(2,2);
   q2(1,1) q2(1,1) q2(1,2) q2(1,2) q2(1,1) q2(1,1) q2(1,2) q2(1,2);
   q2(2,1) q2(2,1) q2(2,2) q2(2,2) q2(2,1) q2(2,1) q2(2,2) q2(2,2);
   q3(1,1) q3(1,1) q3(1,1) q3(1,1) q3(1,2) q3(1,2) q3(1,2) q3(1,2);
   q3(2,1) q3(2,1) q3(2,1) q3(2,1) q3(2,2) q3(2,2) q3(2,2) q3(2,2)];


