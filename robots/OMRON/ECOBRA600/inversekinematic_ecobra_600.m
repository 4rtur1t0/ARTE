%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_ecobra_600(robot, T)	
%   Solves the inverse kinematic problem for the KUKA KR5 scara R350 Z200 scara robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_KUKA_KR5_scara_R350_Z200 returns 4 possible solutions, thus,
%   Q is a 4x4 matrix where each column stores 6 feasible joint values.
%
%   
%   Example code:
%
%   robot=load_robot('kuka', 'KR5_scara_R350_Z200');
%   q = [0 0 0 0];	
%   T = directkinematic(robot, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   for i=1:2,
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
function q = inversekinematic_ecobra_600(robot, T)


fprintf('\nComputing inverse kinematics for the %s robot', robot.name);


%initialize q
q=zeros(4,2);

%Evaluate the DH table to obtain geometric parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%Store geometric parameters
L1=abs(d(1));
L2=abs(a(1));
L3=abs(a(2));

shift = d(3);

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

%find the last rotation for the two possible configurations
q4_1= find_last_rotation(robot,[beta+gamma delta-pi L1-Pz 0], T);
q4_2= find_last_rotation(robot,[beta-gamma pi-delta L1-Pz 0], T);

%Arrange all possible solutions
q=[beta+gamma       beta-gamma;
    delta-pi         pi-delta;
    L1-Pz-shift    L1-Pz-shift ;
    q4_1 q4_2];


% Compute the last rotation
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



