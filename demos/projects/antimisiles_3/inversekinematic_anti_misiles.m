%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the 2dof planar robot
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions or q = [q1 q2] that place the end effectors at the
% position specified by T. Two possible solutions q1 and q2 are returned,
% generally called elbow up and elbow down.
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 08/03/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function q = inversekinematic_anti_misiles(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);

%Initialize q
% q = [q1 q2], atends for only one possible solutions
q=zeros(2,1);

a = eval(robot.DH.a);
d = eval(robot.DH.d);

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Distance of the point to the origin. 
R= sqrt(Py^2+Pz^2);

%Link lengths
L1=abs(d(1));
L2=abs(a(2));

%compute geometric solution

beta = atan2(Py,Px);
gamma=asin((Pz-L1)/L2);

%arrange possible combinations
q =[beta;
    gamma];

%fprintf('\nPx:');disp(Px)
%fprintf('Py:');disp(Py)
%fprintf('Pz:');disp(Pz)
%fprintf('R:');disp(R)

if q(2)<0||q(2)>pi/2
   fprintf('\nERROR: inversekinematic_anti_misiles: unfeasible solution'); 
end

%normalize the solution to [-pi, pi]
%q=atan2(sin(q), cos(q));
