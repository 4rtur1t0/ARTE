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

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);

% initialize q, two possible solutions exist
q=zeros(4,2);

% Evaluate the DH table to obtain geometric parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

% Store the geometric parameters
L1=abs(d(1));
L1p = abs(d(3));
L2=abs(a(1));
L3=abs(a(2));

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);


%Distance of the point to the origin of S0
R = sqrt(Px^2+Py^2);

%Compute angles
gamma = real(acos((L2^2+R^2-L3^2)/(2*R*L2)));
beta = atan2(Py,Px); 
delta = real(acos((L2^2+L3^2-R^2)/(2*L2*L3)));

q1 = beta + gamma + pi;
q1p = beta - gamma + pi;
q2 = -delta;
q2p = delta;
q3 = Pz - L1 - L1p;

% find the last rotation for the two possible configurations
q4= find_last_rotation(robot,[q1 q2 q3 0], T);
q4p= find_last_rotation(robot,[q1p q2p q3 0], T);

% normalize all joints to -pi, pi by atan2
q1 = atan2(sin(q1), cos(q1));
q1p = atan2(sin(q1p), cos(q1p));
q2 = atan2(sin(q2), cos(q2));
q2p = atan2(sin(q2p), cos(q2p));

%Arrange all possible solutions
q=[q1 q1p;
   q2 q2p;
   q3 q3;
   q4 q4p];


% Compute the last rotation, q4 to aling the last axes
function q4 = find_last_rotation(robot, q, T)

U = T(1:3,1);

%Recompute the DH table according to q1, q2 and q3
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%now compute the position/orientation of the system S3
H=eye(4);
for i=1:3
    H=H*dh(theta(i), d(i), a(i), alpha(i));
end

X3=H(1:3,1);
Y3=H(1:3,2);

coseno=X3'*U;
seno=U'*Y3;
%compute the last rotation
q4=atan2(seno,coseno);



