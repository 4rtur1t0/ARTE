%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_IRB6620LX(robot, T)	
%   Solves the inverse kinematic problem for the ABB IRB 6620LX robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_IRB6620LX returns 4 possible solutions, thus,
%   Q is a 6x4 matrix where each column stores 6 feasible joint values.
%
%   
%   Example code:
%
%   robot=load_robot('ABB', 'IRB6620LX');
%   q = [0 0 0 0 0 0];	
%   T = directkinematic(abb, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   for i=1:4,
%        Ti = directkinematic(robot, qinv(:,i))
%   end
%
%	See also DIRECTKINEMATIC.
%
%   Author: Arturo Gil Aparicio
%           Universitas Miguel Hernandez, SPAIN.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio
% 
% This file was written by Miguel Catalan Ba�uls and Jorge Diez Pomares.
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
function q = inversekinematic_irb6620lx(robot, T)

%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);

%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);


%See geometry at the reference for this robot
%L1=d(1);
L3=sqrt((a(3)*a(3))+(d(4)*d(4)));
L2=a(2);
L6=d(6);

A1 = a(1);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% Pm: wrist position
Pm = [Px Py Pz]' - L6*W; 

% first joint: only one solution is feasible. q(1) corresponds
% to a translation
q1=Pm(3);


%solve for q2
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);

%q2_2=solve_for_theta2(robot, [q1+pi 0 0 0 0 0 0], Pm);

%solve for q3
q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);

%q3_2=solve_for_theta3(robot, [q1+pi 0 0 0 0 0 0], Pm);


%Arrange solutions, there are 4 possible solutions so far.
% So far, we have 4 possible solutions. However, for each triplet (theta1, theta2, theta3),
% there exist two more possible solutions for the last three joints, generally
% called wrist up and wrist down solutions. For this reason, 
%the next matrix doubles each column. For each two columns, two different
%configurations for theta4, theta5 and theta6 will be computed. These
%configurations are generally referred as wrist up and wrist down solution
q = [q1         q1         q1        q1      ;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2) ; 
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2);  
     0          0          0         0  ;      
     0          0          0         0 ;       
     0          0          0         0        ];


q=real(q);

%normalize q to [-pi, pi]
%do not normalize q1, it is a translation
q(2,:) = normalize(q(2,:));


% solve for the last three joints
% for any of the possible combinations (theta1, theta2, theta3)
for i=1:2:size(q,2),
    % use solve_spherical_wrist2 for the particular orientation
    % of the systems in this ABB robot
    % use either the geometric or algebraic method.
    % the function solve_spherical_wrist2 is used due to the relative
    % orientation of the last three DH reference systems.
    
    %use either one algebraic method or the geometric 
    %qtemp = solve_spherical_wrist2(robot, q(:,i), T, 1, 'geometric'); %wrist up
    qtemp = solve_spherical_wrist2(robot, q(:,i), T, 1,'algebraic'); %wrist up
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i)=qtemp;
    
    %qtemp = solve_spherical_wrist2(robot, q(:,i), T, -1, 'geometric'); %wrist down
    qtemp = solve_spherical_wrist2(robot, q(:,i), T, -1, 'algebraic'); %wrist down
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i+1)=qtemp;
end

 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry
L3=sqrt((a(3)*a(3))+(d(4)*d(4)));
L2=a(2);

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

alpha = atan2(p1(2), p1(1));
beta = (acos((L2^2+r^2-L3^2)/(2*r*L2)));

if ~isreal(beta)
    disp('WARNING:inversekinematic_irb6620lx: the point is not reachable for this configuration, imaginary solutions'); 
    %gamma = real(gamma);
end

%return two possible solutions
%elbow up and elbow down
%the order here is important and is coordinated with the function
%solve_for_theta3
q2(1) =  alpha + beta; %elbow up
q2(2) = alpha-beta; %elbow down


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function q3 = solve_for_theta3(robot, q, Pm)

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry

L3=sqrt((a(3)*a(3))+(d(4)*d(4)));
L2=a(2);

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(2)^2 + p1(1)^2);

gamma = acos((L2^2 + L3^2 - r^2)/(2*L2*L3));
delta= atan(d(4)/a(3));
if ~isreal(gamma)
   disp('WARNING:inversekinematic_irb6620lx: the point is not reachable for this configuration, imaginary solutions'); 
   %eta = real(eta);
end

%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
q3(1) = delta-(pi-gamma);
q3(2) = (pi-gamma)+delta ;





% %remove complex solutions for q for the q1+pi solutions
% function  qreal = arrange_solutions(q)
% qreal=q;
% 
% %sum along rows if any angle is complex, for any possible solutions, then v(i) is complex
% v = sum(q, 1);
% 
% for i=5:8,
%     if isreal(v(i))
%         qreal=[qreal q(:,i)]; %store the real solutions
%     end
% end
% 
% qreal = real(qreal);