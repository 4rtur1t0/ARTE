%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_IRB910(robot, T)	
%   Solves the inverse kinematic problem for the ABB IRB 910 INV robot
%   (inverted SCARA robot).
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_IRB140 returns 2 possible solutions, thus,
%   Q is a 4x2 matrix where each column stores 4 feasible joint values.
%
%   
%   Example code:
%
%   >>abb=load_robot('ABB', 'IRB910INV');
%   >>q = [0.1 0.1 0.1 0.1];	
%   >>T = directkinematic(abb, q);
%
%   %Call the inversekinematic for this robot
%
%   >> qinv = inversekinematic(abb, T);
%
%   check that all of them are feasible solutions!
%   and every Ti equals T
%
%   for i=1:2
%        Ti = directkinematic(abb, qinv(:,i))
%   end
%
%	See also DIRECTKINEMATIC.
%
%   Author: Arturo Gil Aparicio
%           Universitas Miguel Hernandez, SPAIN.
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
function q = inversekinematic_irb910inv(robot, T)

%initialize q,
%two possible solutions are generally feasible
q=zeros(2,2);

%Evaluate the parameters
d = eval(robot.DH.d);

%See geometry at the reference for this robot
L6=d(6);

%A1 = a(1);

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% P
P = [Px Py Pz]'; 

%first joint, two possible solutions admited: 
% if q(1) is a solution, then q(1) + pi is also a solution
q1=atan2(P(2), P(1));


%solve for q2


%solve for q3




%Arrange solutions, there are 8 possible solutions so far.
% if q1 is a solution, q1* = q1 + pi is also a solution.
% For each (q1, q1*) there are two possible solutions
% for q2 and q3 (namely, elbow up and elbow up solutions)
% So far, we have 4 possible solutions. Howefer, for each triplet (theta1, theta2, theta3),
% there exist two more possible solutions for the last three joints, generally
% called wrist up and wrist down solutions. For this reason, 
%the next matrix doubles each column. For each two columns, two different
%configurations for theta4, theta5 and theta6 will be computed. These
%configurations are generally referred as wrist up and wrist down solution
q = [q1         q1;   
     q2(1)    q2(2);
     q3(1)    q3(2);
     0          0  ];

%leave only the real part of the solutions
q=real(q);

%Note that in this robot, the joint q3 has a non-simmetrical range. In this
%case, the joint ranges from 60 deg to -219 deg, thus, the typical normalizing
%step is avoided in this angle (the next line is commented). When solving
%for the orientation, the solutions are normalized to the [-pi, pi] range
%only for the theta4, theta5 and theta6 joints.

%normalize q to [-pi, pi]




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(robot, q, Pm)






