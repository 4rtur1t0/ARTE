%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q=inversekinematic_Maryland3DOF(robot,T)
% Inverse kinematics for the Maryland 3DOF parallel robot.
%
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions for q = [Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8] 
% that place the end effectors at the position specified by T.
% 
%
% The movements of this robot is the same as the Delta robot, only
% differing in the configuration of the joints.
%
% This robot is divided into three different 3DOF spherical arms.
%
%  
% Any Qi is a 9 value vector that contains: 
% Qi=[theta1 phi11 phi12 theta2 phi21 phi22 theta3 phi31 phi32]'
% being theta1, phi11 phi12 the joint variables of the first arm
%  and  theta2 phi21 phi22 the joint variables of the second arm
%  and  theta3 phi31 phi32 the joint variables of the third arm
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
%   Author: Arturo Gil
%   Date: 18/12/2014
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

function q=inversekinematic_Maryland3DOF(robot,T)

% fprintf('\nComputing inverse kinematics for the %s robot', robot.name);

X=T(1,4); %Get the x position of the final efector.
Y=T(2,4); %Get the y position of the final efector.
Z=T(3,4); %Get the z position of the final efector.

%End effector vector position in the base reference system.
P=[X Y Z 1]';

%This three constant vectors connect the center of the end effector with
%the joint of each of the three limbs
re1=dh(robot.Phi(1), 0, robot.re, pi/2)*[0 0 0 1]';
re2=dh(robot.Phi(2), 0, robot.re, pi/2)*[0 0 0 1]';
re3=dh(robot.Phi(3), 0, robot.re, pi/2)*[0 0 0 1]';

%The three points in base coordinates
P1=P(1:3)+re1(1:3);
P2=P(1:3)+re2(1:3);
P3=P(1:3)+re3(1:3);

%Transformation of each effector joining point into his own arm's reference system
E1p=inv(robot.robot1.T0)*[P1; 1];
E2p=inv(robot.robot2.T0)*[P2; 1];
E3p=inv(robot.robot3.T0)*[P3; 1];


T1=eye(4);
T1(1:3,4)=E1p(1:3);
T2=eye(4);
T2(1:3,4)=E2p(1:3);
T3=eye(4);
T3(1:3,4)=E3p(1:3);


%Inversekinematic_3dofspherical returns elbow in&out joint angle and delta
%angles in q=[theta phi1 phi2], where theta is the active joint and phi1
%and phi2 are passive
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


%Just uncomment the following if you want a wire representation of the arm
% plot_line([0 0 0], P, 'k', 2), hold
% plot_line([0 0 0], P1, 'r', 2)
% plot_line([0 0 0], P2, 'g', 2)
% plot_line([0 0 0], P3, 'b', 2)
% 
% plot_line([0 0 0], robot.robot1.T0*[0 0 0 1]', 'r', 2)
% plot_line([0 0 0], robot.robot2.T0*[0 0 0 1]', 'g', 2)
% plot_line([0 0 0], robot.robot3.T0*[0 0 0 1]', 'b', 2)



function plot_line(p0, p1, color, w)
x0 = p0(1);
y0 = p0(2);
z0 = p0(3);
x1 = p1(1);
y1 = p1(2);
z1 = p1(3);
% Draw a line between p0 and p1
plot3([x0;x1],[y0;y1],[z0;z1], color, 'LineWidth',w);   