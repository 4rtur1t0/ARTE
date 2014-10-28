%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   q = inversekinematic_3dofsphericalb(robot, T)
%
%   Inverse kinematics for the 3dof spherical robot
%   
% robot: structure with arm parameters
% T: homogeneous matrix
% returns: all possible solutions for q = [theta phi1 phi2] that place the end effector at the
% position specified by T. Two possible solutions are returned,
% generally called elbow up and elbow down:
%  
% FIRST column contains the elbow up solution
% SECOND column contains elbow down solution
%
%   This arm is used in the definition of the Delta and Maryland parallel
%   manipulators. Thus, the definition considers that the first joint is an
%   active rotational joint whereas the others two are passive joints.
% 
%   Author: Arturo Gil 
%   email: arturo.gil@umh.es date:   18/12/2013
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
function q = inversekinematic_3dofsphericalb(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot.\n', robot.name);

L1=robot.L1;
L2=robot.L2;

%Obtain the Z coordinate with respect to the robot's base reference system
%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Solve the third passive joint phi2. Note that phi2 is the angle that the
%arm forms with the X0Y0 plane. We can obtain phi2 with its sign from:
phi2=asin(Pz/L2);

%Next, project the arm with length L2 to the X0Y0 plane. When projected, we
%have an arm with length L1 and another with length L2*cos(phi2).
%Now solve as if we had a 2DOF planar rotational arm in the X0Y0 plane. 
L2prima= L2*cos(phi2);


%Distance of the point to the origin in the X0Y0 plane
R=sqrt(Px^2+Py^2);

if R > (L1+L2prima)
   fprintf('\nERROR: inversekinematic_3dofspherical: unfeasible solution'); 
   fprintf('\nERROR: inversekinematic_3dofspherical: the point cannot be reached'); 
end

%compute the solution
beta = atan2(Py,Px); 
gamma = real(acos((L1^2+R^2-L2prima^2)/(2*R*L1)));
delta = real(acos((L1^2+L2prima^2-R^2)/(2*L1*L2prima)));

%arrange possible combinations
% elbow up     elbow down solutions
q =[beta+gamma   beta-gamma;
    delta-pi     pi-delta;
    phi2           phi2];

%normalize the solution to [-pi, pi]
q=atan2(sin(q), cos(q));

                            
                               
 