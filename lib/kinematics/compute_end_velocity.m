%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% V = compute_end_velocity(robot, q, qd)
%
% COMPUTE_END_VELOCITY computes the velocity V of the end effector as a
% function of the joint coordinates q and joint speeds qd.
%
%   If robot.J is defined the Jacobian is evaluated to compute a given
%   velocity. robot.J should be defined as a character string as a function of 
%   the parameters theta, d, a and alpha. The result depends on the
%   definition of J. In consequence, J should be a 6xDOF matrix. For
%   example, for a 2DOF planar arm J is a 6x2 matrix.
%
%   On the other hand, if robot.J is an empty array a conventional Jacobian
%   is computed instead. The compute_conventional_jacobian function is then 
%   called. The result V is then V = [vn wn]' where vn is the
%   linear speed of the end effector and wn is the angular speed, both
%   expressed in the base coordinate system of the robot. This speed (m/s, rad/s)
%   is computed as:
%   
%   V=J*qd
%
%   being qd the joint speeds (rotational or translational) of each joint.   
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
function V = compute_end_velocity(robot, q, qd)

%do not comment the lines below, since J may be 
%evaluated for the theta, d, a and alpha parameters in the first case.
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
%alpha = eval(robot.DH.alpha);


%if we have defined a J matrix for the current robot, just evaluate it
%in this way you can test your computed matrices
if ~isempty(robot.J)
    if robot.debug
        fprintf('\n:compute_end_velocity: Computing end effector speed %s robot with %d DOFs', robot.name, n);
        fprintf('\nUsing user defined J: %s', robot.J);
    end
    
    J = eval(robot.J);
    
    %Now compute speed (linear and angular) using J
    %J encodes the propagation of the angular motion of the joints to the
    % linear and angular motion of the end effector
    V=J*qd';
else%computing J from D-H parameters
    if robot.debug
        disp('\n:compute_end_velocity: Computing conventional Jacobian');
    end    
    J = manipulator_jacobian(robot, q);
    
    % now V = [vn wn]' is the desired linear and angular speed
    % Now compute speed (linear and angular) using J
    % J encodes the propagation of the angular motion of the joints to the
    % linear and angular motion of the end effector
    V = J*qd';
end