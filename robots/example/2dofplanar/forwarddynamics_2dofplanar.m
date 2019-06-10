%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse dynamics for the 2dof planar robot
%
%   tau = inversedynamics_2dofplanar(robot, q, qd, qdd, gc, fext)
%   
%   Where robot stores the kinematic and dynamic parameters for this robot.
%   q: joint positions.
%   qd: joint velocities.
%   qdd: joint accelerations.
%   fext: vector of external forces. Defined in the BASE reference system.
%   let gc=9.81 m/s^2 if the force is acting on the -Y0 direction, or gc =
%   -9.81 m/s^2 if acting on the opposite direction.
%
%   This function just executes the inverse dynamic model for this robot.
%   The equations to compute this dynamic model can be found in:
%   "ROBOT ANALYSIS. The mechanics of Serial and Parallel
%        manipulators". Lung Weng Tsai. John Wiley and Sons, inc. ISBN:
%        0-471-32593-7. page 405.
%   
%   
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 08/03/2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function qdd = forwarddynamics_2dofplanar(robot, q, qd, tau, gc, fext)
a = eval(robot.DH.a);
a1=a(1);
a2=a(2);

m1=robot.dynamics.masses(1);
m2=robot.dynamics.masses(2);

%Express it as a general dynamic function:
%M*qdd + V + G = Q, where Q is a vector of generalized forces or moments
%M is a 2x2 manipulator inertia matrix
M = [(1/3)*m1*a1^2 + m2*(a1^2 + a1*a2*cos(q(2))+(1/3)*a2^2)  m2*((1/2)*a1*a2*cos(q(2))+(1/3)*a2^2);
     m2*((1/2)*a1*a2*cos(q(2))+(1/3)*a2^2)                      (1/3)*m2*a2^2];
 
V = [-m2*a1*a2*sin(q(2))*(qd(1)*qd(2)+(1/2)*qd(2)^2);
            (1/2)*m2*a1*a2*sin(q(2))*qd(1)^2        ];

G = [(1/2)*m1*gc*a1*cos(q(1))+ m2*gc*a1*cos(q(1)) + (1/2)*m2*gc*a2*cos(q(1)+q(2));
         (1/2)*m2*gc*a2*cos(q(1)+q(2))              ];

%assure fext is a column vector
fext=fext(:);

%External forces are propagated to every joint by using the manipulators
%Jacobian
J = manipulator_jacobian(robot, q);   

% %Account for friction by summing in tau.
if robot.dynamics.friction
    for j=1:robot.DOF,
        tau(j) = tau(j) - friction(robot, qd, j);
    end
end

inv(M)
tau
V
G
J'*fext
%Finally use the general equation to compute the torques, considering the
%external forces.
qdd =  inv(M)*(tau - V - G - J'*fext);



