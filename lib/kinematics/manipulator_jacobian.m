%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MANIPULATOR_JACOBIAN(ROBOT, Q) computes the conventional jacobian 
% as a function of the joint coordinates. The algorithm is written as is 
% for educational purposes and translated from:
%
%   Robot Analysis. The mechanics of Serial and parallel manipulators.
%   Lung-Wen Tsai. John Wiley and Sons. ISBN: 0-471-32593-7. Page 186.
%
% Given the conventional Jacobian computed in this way. The end effectors
% velocity defined as V = [vn wn]' can be computed as V = J*qd, where vn 
% and wn are the end effector's linear and angular speed respectively and
% qd is the joint speed. 
%
%   See also DIRECTKINEMATIC.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   26/04/2012
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
function J = manipulator_jacobian(robot, q)

theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);

%number of DOF
n = length(theta);

%base rotation matrix
%R0 = eye(3); 
R0=robot.T0(1:3,1:3); %frequently, it is an identity matrix.

%Z vector on each reference system
z0 = [0 0 1]';


%compute zi vectors
% We first start by computing the zi vectors of each 
% reference system from z0, z1, z2..., z_{n-1}. Note that the 
% last rotational (translational) joint acts on z_{n-1}
%Array to store every z vector
z=[];

%this loop computes vectors from z0 to z_{n-1}
for i=1:n,
    zi = R0*z0;
    
    %store the vector.
    z = [z zi];
    
    %compute the DH transformation matrix from system
    %i-1 to system i
    A = dh(robot, q, i);
    
    %obtain now the global transformation by postmultiplying the
    %rotational part. In the following iteration we include the last DH
    %transformation
    R0=R0*A(1:3,1:3);
end

%compute p{i-1}n* vectors that represent the position of the end effector
%in the {i-1} reference system. 

%Please note that the following code is not optimal (at all), the total
%transformation matrix could be computed in the loop above. However, we
%compute it now using the directkinematic function for this particular
%robot. Again, the DH matrices are computed again.
T = directkinematic(robot, q);

%For example, for a 6DOF robot we start by computing:
%   0p6, the position of T in the base reference system. Next:
%   1p6, the position of the end effector in reference system 1
%   2p6, ...
% every vector will be stored in pn to compute the Jacobian later
pn=zeros(3,robot.DOF); %initialize vector

%initialize with the pos
Ti=robot.T0;

%auxiliar vector that stores the position of the ith reference system,
%starting by the 0 reference system
v=Ti(1:3,4); %[0 0 0]';
for i=1:n,
    %we compute the vector joining the end effector with the reference
    %system i, starting with 0pn, that represents the end effector 
    %coordinates of the base reference system
    pn(:,i) = T(1:3,4)-v(1:3);
    %compute the DH transformation matrix from i-1 to i
    A = dh(robot, q, i);
    
    Ti = Ti*A; 
    %v is the origin of the ith reference system in base coordinates
    v=Ti*[0 0 0 1]';
end


%now compute conventional Jacobian
%The conventional Jacobian is a 6xDOF matrix that allows to compute the 
%linear and angular speed of the end effector as a function of the joint
%speeds qp
J=zeros(6,robot.DOF);
for i=1:n,
    %rotational
    if robot.kind(i)=='R'
        J(:,i) =[cross(z(:,i),pn(:,i)); z(:,i)]; 
    else %translational joint
        J(:,i) =[z(:,i); zeros(3,1)];    
    end   
end
