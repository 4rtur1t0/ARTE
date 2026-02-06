%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_PRR(robot, T)	
%   Solves the inverse kinematic problem for the PRR example robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_PRR returns 2 possible solutions, thus,
%   Q is a 4x4 matrix where each column stores 4 feasible joint values.
%
%   
%   Example code:
%
%   robot=load_robot('example', 'PRR');
%   q = [1 0 0];	
%   T = directkinematic(robot, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   %check that all of them are feasible solutions!
%   %and every Ti equals T
%   for i=1:2,
%        Ti = directkinematic(robot, qinv(:,i))
%   end
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
function q = inversekinematic_PRR(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);

B1 = 1;
 L1 = 1;
 L2 = 1; 
 sphi = T(1,1);
 cphi = T(3,1);
 phi = atan2(sphi, cphi);
 px=T(1,4);
 pz=T(3,4);
  
 k1=px - B1 - L2*sin(phi);
 k2=pz - L2*cos(phi);
 
 a=1;
 b=-2*k2;
 c= k1^2 + k2^2 - L1^2;
 q1 = (-b+sqrt(b^2-4*a*c))/(2*a);
 q1_ = (-b-sqrt(b^2-4*a*c))/(2*a);
 
 q2 = atan2(k1, k2-q1); 
 q2_= atan2(k1, k2-q1_);
 
 q3 = phi-q2;
 q3_ = phi-q2_;
 
 q = [q1 q1_;
     q2 q2_;
     q3 q3_];


