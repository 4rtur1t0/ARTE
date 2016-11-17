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
function Jn = compute_jacobian(robot, q)

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
%CAution, the following functions are evaluated according to the values of q
%compute direct kinematics for serial robotics
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);

%load the position/orientation of the robot's base
T = robot.T0;
%Compute Matrix until joint i-1 (note that no computation is performed for i=1)
for j=1:i-1,
    T=T*dh(theta(j), d(j), a(j), alfa(j));    
end

% obtain z_{i-1}
zi = T(1:3,3);
% obtain o_{i-1}
oi = T(1:3,4);
%a zero vector
zero = zeros(3,1);

%Compute the end effector position
T = directkinematic(robot, q);
on = T(1:3,4);

%rotational joint
if robot.kind(i) == 'R'
    Ji = [cross(zi,(on-oi)); zi];
%prismatic joint
else 
    Ji = [zi; zero];
end

