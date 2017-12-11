% JACOBIAN DEMO

% Copyright (C) 2017, by Arturo Gil Aparicio
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
% along with ARTE.  If not, see <http://www.gnu.odrg/licenses/>.

close all;

fprintf('\nThe demo shows how to compute the end effectors speed as a function of the joint speeds and viceversa')

robot=load_robot('ABB','IRB140');

%First compute the linear and angular speeds of the end effector given the
%joint speeds qd = [1 1 1 1 1 1] rad/s.
% Compute at joint position q = [0.1 0.1 0.1 0.1 0.1 0.1] rad
q = [0.1 0.1 0.1 0.1 0.1 0.1]'; 
qd = [1 1 1 1 1 1]';
% The result V is  [Vx Vy Vz Wx Wy Wz] [m/s m/s m/s rad/s rad/s rad/s]
V = compute_end_velocity(robot, q, qd)
 
% Now, at the same position q, we would like to compute the join speeds qd
% that will bring the end effector to the speed V. The result should match
% the values in qd as defined earlier
qd = compute_joint_velocity(robot, q, V)


%SINGULARITIES
% The abb irb140 shows a singularity point at q = [0 0 0 0 0 0]. This means 
% that the manipulator Jacobian J cannot be inverted (det(J)=0). If we 
% repeat the former instructions for q = [0 0 0 0 0 0] we get a Warning 
% indicating that J is badly conditioned and an unaccurate result.  
% (Uncomment the following 4 lines to test this case)
%q = [0 0 0 0 0 0]; 
%qd = [1 1 1 1 1 1];
%V = compute_end_velocity(robot, q, qd)
%qd = compute_joint_velocity(robot, q, V)

