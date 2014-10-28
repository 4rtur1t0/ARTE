%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  [Qdd]= ACCEL(ROBOT, Q, Qd, TORQUE)
%
%  Computes the instantaneous acceleration Qdd at each joint for the robot ROBOT
%  asuming that the robot is at a state defined by joint position Q0 and velocity
%  Qd0 and a TORQUE is applied.
%
%  Returns the a vector os instantaneous accelerations at each joint.
%
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
function qdd = accel(robot, q, qd, torque, g)

if ~exist('g','var')
    g=[0  0  9.81]'; %default gravity
end


n = robot.DOF;

%   compute the manipulator intertia matrix
%   to do so, compute the torques resulting from unit acceleration of
%   each joint without gravity
M=[];
for i=1:robot.DOF,
    %cirshift([1 0 0 0 0 0], i-1) results in acceleration varying from
    % [1 0 0 0 0 0], [0 1 0 0 0 0], [0 0 1 0 0 0] etc
    t = inversedynamic(robot, q', zeros(1,n), circshift([1 zeros(1,n-1)]',i-1), [0  0  0]', [0 0 0 0 0 0]');
    %t = inversedynamic(robot, q', zeros(1,n), circshift([1 0 0 0 0 0]',i-1), [0  0  0]', [0 0 0 0 0 0]');
    M = [M t];
end

% use inverse dynamic model to compute torques
% in the state defined by position q and velocity qd under
% the action of gravity.
tau = inversedynamic(robot, q', qd', zeros(1,n), g, [0 0 0 0 0 0]');


qdd = inv(M)*(torque(:) - tau);