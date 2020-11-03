%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_UNI1(robot, T)	
%   Solves the inverse kinematic problem for the UNIMATE 1 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_UNI1 returns 1 possible solution, thus,
%   Q is a 6x1 matrix.
%
%   
%   Example code:
%
%   robot=load_robot('UNIMATE', 'UNIMATE1');
%   q = [0 0 0 0 0 0];	
%   T = directkinematic(robot, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   
%   Ti = directkinematic(robot, qinv(:))
%   
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
function q = inversekinematic_uni1(robot, T)

%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);

%Evaluate the parameters
d = eval(robot.DH.d);

%See geometry at the reference for this robot
L0=d(1);
L1=d(5);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

% Compute the position of the wrist, being W the Z component of the end effector's system
z5 = T(1:3,3);

% Pm: wrist position
Pm = [Px Py Pz]' - L1*z5; 
Pmx = Pm(1);
Pmy = Pm(2);
Pmz = Pm(3);

% first joint, single solution
q1 = atan2(Pmy, Pmx);
q = [q1 0 0 0 0];
A01=dh(robot, q, 1);
Pm1 = inv(A01)*[Pm; 1];
pmx1 = Pm1(1);
pmy1 = Pm1(2);
q2 = atan2(pmy1, pmx1);
% compute q3 as the norm of Pm
q3 = sqrt(Pmx^2+Pmy^2+(Pmz-L0)^2);
q = [q1 q2 q3 0 0];
% now compute q4
A01=dh(robot, q, 1);
A12=dh(robot, q, 2);
A23=dh(robot, q, 3);
A03 = A01*A12*A23;
z4 = z5;
x3 = A03(1:3,1);
y3 = A03(1:3,2);
cq4 = dot(-y3, z4);
sq4 = dot(z4, x3);
q4 = atan2(sq4, cq4);
% COMPUTING Q5
q = [q1 q2 q3 q4 0];
A34=dh(robot, q, 4);
A04 = A03*A34;
x4 = A04(1:3,1);
y4 = A04(1:3,2);
x5 = T(1:3,1);
cq5 = dot(x4, x5);
sq5 = dot(x5, y4);
q5 = atan2(sq5, cq5);
q = [q1 q2 q3 q4 q5];

%leave only the real part of the solutions
q=real(q);

