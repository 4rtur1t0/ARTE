%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the Delta 3DOF parallel robot
% 
%
% DIRECTKINEMATIC_3RRR(robot, Q)
% Q: joints vector with structure: 
% [theta1 beta1 delta1 theta2 beta2 delta2 theta3 beta3 delta3]
% theta is the active joint´s vector
% beta is the passive joint's vector in the arm plan
% theta is the passive joint's vector out of the arm plan
% robot: structure with arm parameters
% returns: T solution. 
% shows T1 and T2 with the 2 posible solution. T1 it´s unachievable
% 
%
% 
% 
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

function T=directkinematic_DELTA3DOF(robot,q)

%Robot parameters 

L1=robot.L1;
L2=robot.L2;
d1=robot.d1;
d2=robot.d2;

alpha=robot.alpha;

%Base´s rotationals joints position
A1=[d1*cos(alpha(1)),d1*sin(alpha(1)),0];
A2=[d1*cos(alpha(2)),d1*sin(alpha(2)),0];
A3=[d1*cos(alpha(3)),d1*sin(alpha(3)),0];

%Cardans joints position. Union of both arms
B1=[(d1+ L1*cos(q(1)))*cos(alpha(1)),(d1+ L1*cos(q(1)))*sin(alpha(1)),-L1*sin(q(1))];
B2=[(d1+ L1*cos(q(4)))*cos(alpha(2)),(d1+ L1*cos(q(4)))*sin(alpha(2)),-L1*sin(q(4))];
B3=[(d1+ L1*cos(q(7)))*cos(alpha(3)),(d1+ L1*cos(q(7)))*sin(alpha(3)),-L1*sin(q(7))];


%Displacement of the cardan position that is the center of the
%circunference needed to solve de direct kinematic
B1_displaced=[B1(1) - (d2*cos(alpha(1))), B1(2) - (d2*sin(alpha(1))), B1(3)];
B2_displaced=[B2(1) - (d2*cos(alpha(2))), B2(2) - (d2*sin(alpha(2))), B2(3)];
B3_displaced=[B3(1) - (d2*cos(alpha(3))), B3(2) - (d2*sin(alpha(3))), B3(3)];

[X,Y,Z] = get_end_position(B1_displaced, B2_displaced, B3_displaced, L2);

T1=eye(4);
T2=eye(4);
T1(1:3,4)=[X(1), Y(1), Z(1)]';
T2(1:3,4)=[X(2), Y(2), Z(2)]';

%Yes, returning just the downwards solution
T=T2;

end