%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_RRR(robot, T)	
%   Solves the inverse kinematic problem for the KUKA KR180 R2100 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC__KUKA_KR180_R2100 returns 8 possible solutions, thus,
%   Q is a 6x8 matrix where each column stores 6 feasible joint values.
%
%   
%   Example code:
%
%   robot=load_robot('kuka', 'KR180_R2100');
%   q = [0 0 0 0 0 0];	
%   T = directkinematic(robot, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   for i=1:8,
%        Ti = directkinematic(robot, qinv(:,i))
%   end
%	See also DIRECTKINEMATIC.
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
function q = inversekinematic_RRR(robot, T)

%initialize q,
%eight possible solutions are generally feasible
q=zeros(3,4);

%Note that T is referred to Matlab's universal reference system. Therefore,
%we will change the reference system to ours, which is given in the robot's 
%documentation. 

%T viene dada en el sistema de referencia de matlab, por lo que hay que
%ponerla en funci�n de nuestro sistema de referencia
%T=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]*T;

% %Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

L1=abs(d(1));

%Compute the position of the wrist, being W the Z component of the end effector's system
P = T(1:3,4);


%first joint, two possible solutions admited: 
% if q(1) is a solution, then q(1) + pi is also a solution
q1=atan2(P(2), P(1));
q1_ = q1 + pi;
% normalize to -pi, pi
q1_ = atan2(sin(q1_), cos(q1_));

%solve for q2
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], P);
%the other possible solution is q1 + pi
q2_2=solve_for_theta2(robot, [q1_ 0 0 0 0 0 0], P);

%solve for q3
q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], P);
%solver for q3 for both cases
q3_2=solve_for_theta3(robot, [q1_ 0 0 0 0 0 0], P);

%compose all the 4 possible solutions.
q = [q1         q1         q1_       q1_    ;   
     q2_1(1)    q2_1(2)    q2_2(1)   q2_2(2);
     q3_1(1)    q3_1(2)    q3_2(1)   q3_2(2)];
%leave only the real part of the solutions
q=real(q);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, P)

%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%See geometry
L2=abs(a(2));
L3=abs(a(3));

%The inverse kinematic problem can be solved as in the IRB 140 (for example)

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[P; 1];

R = sqrt(p1(1)^2 + p1(2)^2);
beta = atan2(p1(2), p1(1));
gamma = real(acos((L2^2+R^2-L3^2)/(2*R*L2)));

%return two possible solutions
%elbow up and elbow down
%the order here is important and is coordinated with the function
%solve_for_theta3
q2(1) =  beta + gamma; %elbow up
q2(2) =  beta - gamma; %elbow down


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(robot, q, P)

%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%See geometry
L2=abs(a(2));
L3=abs(a(3));

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[P; 1];
R = sqrt(p1(1)^2 + p1(2)^2);
eta = real(acos((L2^2 + L3^2 - R^2)/(2*L2*L3)));

%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
q3(1) =  eta - pi; % elbow up
q3(2) = pi - eta; 





