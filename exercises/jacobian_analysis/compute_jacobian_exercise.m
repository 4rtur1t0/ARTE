%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	WRITE A FUNCTION THAT COMPUTES THE JACOBIAN OF A ROBOT
%
%   Jn = compute_jacobian(robot, q)
%   
%   The method exposed here as a solution is translated from the method in:
%   
%   "Robot Modeling and Control". Mark W. Spong, Seth Hutchinson, M. Vidyasagar
%   Ed. Wiley
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2016, by Arturo Gil Aparicio
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
function Jn = compute_jacobian_exercise(robot, q)

%Initialize J
Jn = [];
for i=1:robot.DOF,
   Ji = jacobian_submatrix(robot, q, i);
   % Add submatrix
   Jn = [Jn Ji];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Computes the value of the submatrix Ji which accounts for the
%   contribution of joint i (either rotational or prismatic) in the
%   rotational and translational speed of the end effector.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Ji = jacobian_submatrix(robot, q, i)
%Caution, the following functions are evaluated according to the values of q
%compute direct kinematics for serial robotics
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);

%load the position/orientation of the robot's base
T = robot.T0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: 
% Compute Matrix until joint i-1 (note that no computation is performed for i=1)
% use the dh(theta(j), d(j), a(j), alfa(j)) function
if i==1
   z0=[0 0 1]';
   o0 = [0 0 0]';
   T=directkinematic(robot, q);
   on = T(1:3,4);

    Ji=[cross(z0,on-o0); 
    z0];
end

if i==2
    A01 = dh(theta(1),d(1),a(1), alfa(1))
    z1 = A01(1:3,3);
    T=directkinematic(robot, q);
    on = T(1:3,4);
    o1= A01(1:3,4);
    
    Ji=[cross(z1,on-o1); 
    z1];
end

if i==3
    A01 = dh(theta(1),d(1),a(1), alfa(1))
    A12 = dh(theta(2),d(2),a(2), alfa(2))
    A02 = A01*A12;
    z2 = A02(1:3,3);
    T=directkinematic(robot, q);
    on = T(1:3,4);
    o2= A02(1:3,4);
    
    Ji=[cross(z2,on-o2); 
    z2];
end


%

% store the result in variable T
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


