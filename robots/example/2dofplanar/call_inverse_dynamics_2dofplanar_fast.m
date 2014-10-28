%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  tau = call_inverse_dynamics_2dofplanar_fast(input)
%  Auxiliar function for the simulink model SIMULATE_ROBOT_2DOF.
%  Rearranges the inputs coming from the simulink model and calls the
%  function inverse_dynamics_2dof_planar_fast.
%  As a result the instantaneous acceleration at each joint is returned.
%
%  See also inverse_dynamic
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
function tau = call_inverse_dynamics_2dofplanar_fast(input)

global robot


q = input(1:2);        % Input joint positions
qd   = input(3:4);	   % input joint speeds
qdd  = input(5:6);   % Input joint accelerations
fe=[0 0];

% Compute torques
tau = inversedynamics_2dofplanar_fast(robot, q, qd, qdd, fe);


