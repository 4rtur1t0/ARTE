%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse dynamics for the 1dof planar robot
%
%   tau = inversedynamics_1dofplanar(robot, q, qd, qdd, fext)
%   
%   Where robot stores the kinematic and dynamic parameters for this robot.
%   q: joint positions.
%   qd: joint velocities.
%   qdd: joint accelerations.
%   fext: vector of external forces. Defined in the last reference system.
%
%   This function just executes the inverse dynamic model for this robot.
%   The equations to compute this dynamic model can be found in:
%   "ROBOT ANALYSIS. The mechanics of Serial and Parallel
%        manipulators". Lung Weng Tsai. John Wiley and Sons, inc. ISBN:
%        0-471-32593-7. page 405.
%   
%   
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 23/11/2016
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
function tau = exercise_inv_dynamics_1dofplanar(robot, q, qd, qdd, g, fext)
a = eval(robot.DH.a);
a1=a(1);

g=abs(g);
m=robot.dynamics.masses(1);

%In this case we must define the Inertia with respect to the rotating axis.
J = (1/3)*m*a1^2;
tau = J*qdd + m*g*a1*cos(q(1))/2;






