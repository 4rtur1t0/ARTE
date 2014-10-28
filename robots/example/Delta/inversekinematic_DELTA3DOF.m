%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the Delta 3DOF parallel robot
% The Delta robot is divided into three different 2DOF arms.
%
% INVERSEKINEMATIC_DELTA3DOF(robot, T)
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions for q = [Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8] 
% that place the end effectors at the position specified by T. 
% Any Qi is a 9 value vector that contains: 
% Qi=[theta1 beta1 delta1 theta2 beta2 delta2 theta3 beta3 delta3]'
% being theta1, beta1, delta1 the joint variables of the first arm
%  and  theta2, beta2, delta2 the joint variables of the second arm
%  and  theta3, beta3, delta3 the joint variables of the third arm
%
% Eight possible solutions q = [Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8],
% generally called elbow out and elbow in in eight different combinations for
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
%       1: elbow out solution
%       0: elbow in solution
%   Author: Ángel Rodríguez
%   Date: 18/12/2013
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

%Inverse kinematics for a parallel delta robot
%This function povides the angle solution for a TCP gave
%Arms length can be provided. If it's not default lengts will be used
%Base and efector size is fixed.
%Elbow = 0 means that the elbow's solution is out
%Can be placed as the thrird input argument

function q=inversekinematic_DELTA3DOF(robot,T)

% fprintf('\nComputing inverse kinematics for the %s robot', robot.name);

X=T(1,4); %Get´s the x positión of the final efector.
Y=T(2,4); %Get´s the y positión of the final efector.
Z=T(3,4); %Get´s the z positión of the final efector.

%Rotational joints angle
alpha=robot.alpha;

%Base and effector parameters
d1=robot.d1;
d2=robot.d2;

TX=robot.TX;

%Effector apex position depending on X, Y & Z
E1=[X+d2*cos(alpha(1)),Y+d2*sin(alpha(1)),Z];
E2=[X+d2*cos(alpha(2)),Y+d2*sin(alpha(2)),Z];
E3=[X+d2*cos(alpha(3)),Y+d2*sin(alpha(3)),Z];

%Transformation of each effector's apex into his arm's reference system
E1p=inv(robot.robot1.T0)*[E1,1]';
E2p=inv(robot.robot2.T0)*[E2,1]';
E3p=inv(robot.robot3.T0)*[E3,1]';

T1=eye(4);
T1(1:3,4)=E1p(1:3);
T2=eye(4);
T2(1:3,4)=E2p(1:3);
T3=eye(4);
T3(1:3,4)=E3p(1:3);

%Sintaxis for the ARTE library
%Must return both posibilities and delta angles in q

%Inversekinematic_3dofspherical returns elbow in&out joint angle and delta
%angles in q=[in out delta1 delta2]
q1=inversekinematic(robot.robot1,T1);
q2=inversekinematic(robot.robot2,T2);
q3=inversekinematic(robot.robot3,T3);

%Returns 8 posibilities
%If is used q(1) elbow IN solution will be taked
%If is used q(2) elbow OUT solution will be taked
%[theta1 beta1 delta1 theta2 beta2 delta2 theta3 beta3 delta3]'

q=[q1(:,1), q1(:,1), q1(:,1), q1(:,1), q1(:,2), q1(:,2), q1(:,2), q1(:,2); 
   q2(:,1), q2(:,1), q2(:,2), q2(:,2), q2(:,1), q2(:,1), q2(:,2), q2(:,2);
   q3(:,1), q3(:,2), q3(:,1), q3(:,2), q3(:,1), q3(:,2), q3(:,1), q3(:,2)];
   
% Saves for final configuration the angles for elbow out
qf=q(:,1);

if qf(2)>=pi ||  qf(5)>=pi || qf(8)>=pi
    fprintf('\nWARNING: inversekinematic_DELTA: unfeasible solution. The arm2 impacts arm 1.\n');
end
if qf(1)>pi/2 | qf(1)<-pi | qf(4)>pi/2 | qf(4)<-pi | qf(4)>pi/2 | qf(4)<-pi 
    fprintf('\nWARNING: inversekinematic_3dofplanar: unfeasible solution. The arm 1 impacts the robot´s base.\n');
end


end