%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_IRB760(robot, T)	
%   Solves the inverse kinematic problem for the ABB IRB 760 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_IRB760 returns only 1 possible solution, thus,
%   Q is a 5x1 matrix where each column stores 5 feasible joint values.
%   Beware that the 4th axis is a combination of q(2) and q(3).
%
%   
%   Example code:
%
%   >>abb=load_robot('ABB', 'IRB760');
%   >>q = [0 0 0 0 0 0];	
%   >>T = directkinematic(abb, q);
%   %Call the inversekinematic for this robot
%   >>qinv = inversekinematic(abb, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%
%   for i=1:8,
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
function q = inversekinematic_irb760(robot, T)

%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);

% %Evaluate the parameters
a = eval(robot.DH.a);
d = eval(robot.DH.d);
L6=d(5);
L4=a(4);



%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% Pm: wrist position
Pm = [Px Py Pz]';%' - L6*W; 

%first joint, two possible solutions admited: 
% if q(1) is a solution, then q(1) + pi is also a solution
q1=atan2(Pm(2), Pm(1));

%compute the wrist, that, in this case is the origin of the
% system X3Y3Z3
Pm = [Px Py Pz]' - L6*W -L4*[cos(q1) sin(q1) 0]';

%solve for q2
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);
%the other possible solution is q1 + pi
%q2_2=solve_for_theta2(robot, [q1+pi 0 0 0 0 0 0], Pm);

%solve for q3
q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);
%solver for q3 for both cases
%q3_2=solve_for_theta3(robot, [q1+pi 0 0 0 0 0 0], Pm);


%the next matrix doubles each column. For each two columns, two different
%configurations for theta4, theta5 and theta6 will be computed. These
%configurations are generally referred as wrist up and wrist down solution
q = [q1                          q1   ;   
     q2_1(1)                   q2_1(2);
     q3_1(1)                   q3_1(2);
    -q2_1(1)-q3_1(1)          -q2_1(2)-q3_1(2);
      0                           0];

%leave only the real part of the solutions
q=real(q);

%normalize q to [-pi, pi]
q(1,:) = normalize(q(1,:));
q(2,:) = normalize(q(2,:));

%leave only the elbow up solution, which is the only physically feasible
q=q(:,1);

T01=dh(robot, q, 1);
T12=dh(robot, q, 2);
T23=dh(robot, q, 3);
T34=dh(robot, q, 4);
        
T04=T01*T12*T23*T34;

y4 = T04(1:3,2);
x4 = T04(1:3,1);

sq5 = dot(T(1:3,1), y4);	% Vector orientación n: T(1:3,1)
cq5 = dot(T(1:3,1), x4);	% Vector orientación o: T(1:3,2)
q5  = atan2(sq5, cq5);

% Joint positions vector 
%q(4)=-q(2)-q(3);
q(5)=q5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)

%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%See geometry
L2=a(2);
L3=a(3);


%The inverse kinematic problem can be solved as in the IRB 140 (for example)

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(-p1(2), p1(1));
gamma = real(acos((L2^2+r^2-L3^2)/(2*r*L2)));

%return two possible solutions
%elbow up and elbow down
%the order here is important and is coordinated with the function
%solve_for_theta3
q2(1) = pi/2 - beta - gamma; %elbow up
q2(2) = pi/2 - beta + gamma; %elbow down


% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % solve for third joint theta3, two different
% % % solutions are returned, corresponding
% % % to elbow up and down solution
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % function q3 = solve_for_theta3(robot, q, Pm)
% % 
% % %Evaluate the parameters
% % theta = eval(robot.DH.theta);
% % d = eval(robot.DH.d);
% % a = eval(robot.DH.a);
% % alpha = eval(robot.DH.alpha);
% % 
% % %See geometry
% % L2=a(2);
% % L3=d(4);
% % A2=a(3);
% % 
% % %See geometry of the robot
% % %compute L4
% % L4 = sqrt(A2^2 + L3^2);
% % 
% % %the angle phi is fixed
% % phi=acos((A2^2+L4^2-L3^2)/(2*A2*L4));
% % 
% % %given q1 is known, compute first DH transformation
% % T01=dh(robot, q, 1);
% % 
% % %Express Pm in the reference system 1, for convenience
% % p1 = inv(T01)*[Pm; 1];
% % 
% % r = sqrt(p1(1)^2 + p1(2)^2);
% % 
% % eta = real(acos((L2^2 + L4^2 - r^2)/(2*L2*L4)));
% % 
% % %return two possible solutions
% % %elbow up and elbow down solutions
% % %the order here is important
% % q3(1) = pi - phi- eta; 
% % q3(2) = pi - phi + eta; 
% 
% 
% %return two possible solutions
% %elbow up and elbow down solutions
% %the order here is important
% q3(1) = pi/2 - eta;
% q3(2) = eta - 3*pi/2;



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
%The parameters are obtained, in this particular robot
% from the a vector
L2=a(2);
L3=a(3);

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

eta = (acos((L2^2 + L3^2 - r^2)/(2*L2*L3)));

if ~isreal(eta)
   disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
   %eta = real(eta);
end

%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
q3(1) = pi/2 - eta;
q3(2) = eta - 3*pi/2;



