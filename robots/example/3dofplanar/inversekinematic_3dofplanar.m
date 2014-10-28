%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the 3dof planar robot
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions or q = [q1 q2] that place the end effectors at the
% position specified by T. Two possible solutions q1 and q2 q3 and q4 are returned,
% generally called elbow up and elbow down, combined with 
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
function q = inversekinematic_3dofplanar(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);


%Initialize q
% q = [q1 q2 q3 q4], atends for two possible solutions
q=zeros(3,4);

a = eval(robot.DH.a);

%Link lengths
L1=abs(a(1));
L2=abs(a(2));
L3=abs(a(2));


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Q=T(1:3,4);

%find angle Phi
x3 = T(1:3,1);

cphi = x3'*[1 0 0]';
sphi = x3'*[0 1 0]';
phi = atan2(sphi, cphi);


%Find point P from P and vector (nx, ny, nz)
P = Q - a(3)*T(1:3,1);

%Distance of the point to the origin. 
R= sqrt(P(1)^2+P(2)^2);

if R > (L1+L2)
   disp('\ninversekinematic_3dofplanar: unfeasible solution. The point cannot be reached'); 
end

%compute geometric solution
beta = atan2(P(2),P(1)); 
gamma = real(acos((L1^2+R^2-L2^2)/(2*R*L1)));
delta = real(acos((L1^2+L2^2-R^2)/(2*L1*L2)));

%arrange possible combinations for q(1) and q(2) 
%elbow down     elbow up solutions
q =[beta+gamma beta-gamma;
    delta-pi   pi-delta];

%in this case, phi = q(1) + q(2) + q(3) and
%q(3) can be computed as q(3) = phi - q(1) - q(2) 
%corresponding to each of the previous solutions, a unique q(3) can be
%computed for each case
for i=1:2, %iterate through columns 
    q(3,i) = phi - q(1,i) - q(2,i); 
end


