%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_KUKA_KR120_R3200_PA(robot, T)	
%   Solves the inverse kinematic problem for the KUKA KR120 R3200 PA robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC__KUKA_KR120_R3200_PA returns 1 possible
%   solutions.
%   
%   Example code:
%
%   robot=load_robot('kuka', 'KR120_R3200_PA');
%   q = [0 0 0 0];	
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
function q = inversekinematic_kuka_KR120_R3200_PA(robot, T)

%initialize q,
q=zeros(4,1);

% %Evaluate the parameters
% theta = eval(robot.DH.theta);
d = eval(robot.DH.d);

L6=abs(d(5));
 

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
Rt1 = T(1,1);
Rt2 = T(1,2);

%Compute the position of the wrist, being W the Z component of the end effector's system

% Pm: wrist position
Pm = [Px Py Pz]';
l2 = 1.35;
l3 = 1.22;

q1=atan2(Pm(2), Pm(1));

Pm = [Px-0.280*cos(q1) Py-0.280*sin(q1) Pz]';

R = sqrt(Pm(1)^2 + Pm(2)^2) - 0.350;
p = (Pm(3) - 0.675 + 0.25);
alfa = atan2(p,R);
aux = (R/cos(alfa));
beta = acos((l2^2 + aux^2 - l3^2)/(2*l2*aux));
q2 = pi/2 - alfa - beta;
gamma = acos((l2^2 + l3^2 - aux^2)/(2*l2*l3));
q3 = pi/2 - gamma;
q4 = 0;

if (Rt1 > 0) && (Rt2 < 0)
    %Primer cuadrante
    q4 = -asin(Rt2) + q1;
elseif (Rt1 <= 0) && (Rt2 <= 0)
    %Segundo cuadrante
    q4 = -asin(Rt2) + pi/2 + q1;
elseif (Rt1 < 0) && (Rt2 > 0)
    %Tercer cuadrante
    q4 = -asin(Rt2) - pi/2 + q1;
elseif (Rt1 >= 0) && (Rt2 >= 0)
    %Cuarto cuadrante
    q4 = -asin(Rt2) + q1;
end

q = real([q1;   
     q2;
     q3;
     -q2-q3;
     q4;
     ]);
 
 %q = roundn(q,-4);