%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  qdd = call_inverse_dynamics(input)
%  Auxiliar function for the simulink model inverse_dynamics.
%  Rearranges the inputs coming from the simulink model and calls the
%  function inversedynamics.
%  As a result the instantaneous torques at each joint are returned.
%
%  See also ACCEL, INVERSEDYNAMICS.
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
% You should have received a copy of the GNU Lesser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function tau = call_inverse_dynamics(input)

global robot

%set friction to zero
robot.friction = 0;

q = input(1:2);   % Input position at each joint
qd = input(3:4);	   % Joint speeds
qdd = input(5:6);	   % Joint accelerations

%gravity should act on the negative direction with Y0
g=[0 0 0]'; %m/s^2
fext = [0 0 0 0 0 0]';

% Compute acceleration
tau = inversedynamic(robot, q, qd, qdd, g, fext);

