%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EXERCISE: WRITE THE LAGRANGE FORMULATION FOR A 3DOF SPHERICAL 
% ROBOT with RRR joints
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
function dynamics_symbolic_lagrange_3dof_RRR_spherical

robot = load_robot('example', 'example_RRR');

syms q1 q2 q2 qd1 qd2 qd3 L1 L2 L3 m1 m2 m3 I1 I2 I3 real
q = [q1 q2 q3];
qd = [qd1 qd2 qd3];
n=3;
% the Jacobian of the center of mass
Jvc1 = compute_jacobian_vci(robot, q, 1);
Jvc2 = compute_jacobian_vci(robot, q, 2);
Jvc3 = compute_jacobian_vci(robot, q, 3);


M = m1*Jvc1'*Jvc1+m2*Jvc2'*Jvc2 + m3*Jvc3'*Jvc3 
M = M + [I1 0; 0 0] + [I2 I2; I2 I2];
M = simplify(M)


V = []
% compute Vi
for i=1:n
    disp('Computing V')
    i
    Vi = 0;
    for j=1:n
        for k=1:n
            fprintf('\n%d, %d, %d', i, j, k);
            a = diff(M(i, j), q(k));
            b = -(0.5)*diff(M(j, k), q(i));            
            (a+b)*qd(j)*qd(k)
            Vi = Vi + (a+b)*qd(j)*qd(k);
        end
    end
    Vi
    Vi = simplify(Vi);
    V = [V; Vi];
end
V


function Jvci = compute_jacobian_vci(robot, q, linki)
% find oci (oc1)
z0i = [0 0 1];
R = eye(3);
Jvci = []
Jwi = []
for i=1:linki
    
end





