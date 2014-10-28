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
function q = inversekinematic_2dofplanar(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);


%Initialize q
% q = [q1 q2], atends for two possible solutions
q=zeros(2,2);

% theta = eval(robot.DH.theta);
% d = eval(robot.DH.d);
 a = eval(robot.DH.a);
% alpha = eval(robot.DH.alpha);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Distance of the point to the origin. 
R= sqrt(Px^2+Py^2);

%Link lengths
L1=abs(a(1));
L2=abs(a(2));

if R > (L1+L2)
   fprintf('\nERROR: inversekinematic_2dofplanar: unfeasible solution'); 
end

%compute geometric solution
beta = atan2(Py,Px); 
gamma = real(acos((L1^2+R^2-L2^2)/(2*R*L1)));
delta = real(acos((L1^2+L2^2-R^2)/(2*L1*L2)));

%arrange possible combinations
%elbow up     elbow down solutions
q =[beta+gamma beta-gamma;
    delta-pi pi-delta];

