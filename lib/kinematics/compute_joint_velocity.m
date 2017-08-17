%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMPUTE_JOINT_VELOCITY computes the velocity of the joint given the velocity
% vector V of the end effector
%
%   If robot.J is defined the Inverse Jacobian is evaluated to compute a given
%   velocity. The result depends on the definition of J.
%
%   On the other hand, if robot.J is an empty array a conventional Jacobian
%   is computed instead. The resulting joint speeds is computed as
%   qd = inv(J)*V, where V is the velocity vector expressed in the base reference frame.
%
%   See also MANIPULATOR_JACOBIAN.
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
function vq = compute_joint_velocity(robot, q, V)

theta = eval(robot.DH.theta);
% d = eval(robot.DH.d);
a = eval(robot.DH.a);
% alpha = eval(robot.DH.alpha);

n=length(theta); %# number of DOFs

%use user provided J or compute jacobian
if ~isempty(robot.J)
    J = eval(robot.J);   
    if robot.debug
        fprintf('\nComputing end effector speed for the %s robot with %d DOFs', robot.name, n);
        fprintf('\n:lib/kinematics/compute_joint_velocity: Using user defined J', robot.J);    
    end
else%computing J from D-H parameters
    if robot.debug
        fprintf('\nComputing end effector speed for the %s robot with %d DOFs', robot.name, n);
        disp('\n:lib/kinematics/compute_joint_velocity: Computing conventional Jacobian');
    end
    J = manipulator_jacobian(robot, q);  
end
%assure V is a column vector
V = V(:);
%Solve inverse kinematic problem with the inversion of J
vq = J\V;%inv(J)*V';