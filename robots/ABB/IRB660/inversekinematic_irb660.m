%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_IRB660(robot, T)	
%   Solves the inverse kinematic problem for the ABB IRB 140 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_IRB660 just one solution.
%
%   
%   Example code:
%
%   >>abb=load_robot('ABB', 'IRB660');
%   >>q = [0 0 0 0 0 0];	
%   >>T = directkinematic(abb, q);
%
%   %Call the inversekinematic for this robot
%
%   >> qinv = inversekinematic(abb, T);
%
%   check that all of them are feasible solutions!
%   and every Ti equals T
%
%   for i=1:8,
%        Ti = directkinematic(abb, qinv(:,i))
%   end
%
%	See also DIRECTKINEMATIC.
%
%   Author: Andrés López Recas
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
function q = inversekinematic_irb660(robot, T)

%initialize q,
%one possible solution are generally feasible
q=zeros(5,1);

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry at the reference for this robot
 L5=d(5);
 a4=a(4);

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% Pm: wrist position
 Pm = [Px Py Pz]' - L5*W;

%first joint has a range from 300 to -300 degrees, so there is only one solution in the range between -60 and 60 
q1=atan2(Py, Px);

%The joint 4 only is used to correct the Z component of the end effector's
%system to be aligned with the first join, so a correction of Px is needed
%to calculate the other q
Pm = Pm - [cos(q1)*a4 sin(q1)*a4 0]';

%Q4 is only a rotation in Z axis
q4=acos(T(1,1));

%solve for q2
q2=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);

%solve for q3
q3=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);


%Arrange solutions, there are only one possible solutions so far.

q = [q1       ;   
     q2       ;
     q3       ;
     -(q2+q3) ;
     q4      ];

%leave only the real part of the solutions
q=real(q);


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
L2=a(2);
L3=a(3);

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

h=sqrt(p1(1)^2+p1(2)^2)

beta = acos((L2^2+h^2-L3^2)/(2*L2*h));
gamma = atan2(p1(2),p1(1));

% b=rad2deg(beta)
% g=rad2deg(gamma)

if ~isreal(gamma)
    disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
    %gamma = real(gamma);
end

%return two possible solutions
%elbow up and elbow down
%the order here is important and is coordinated with the function
%solve_for_theta3
q2 = pi/2 - beta + gamma; %elbow up



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
L2=a(2);
L3=a(3);

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

h=sqrt(p1(1)^2+p1(2)^2)

eta = acos((L2^2+L3^2-h^2)/(2*L2*L3));

if ~isreal(eta)
   disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
   %eta = real(eta);
end

%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
 q3 = pi/2 - eta;
% q3(2) = eta - 3*pi/2;



