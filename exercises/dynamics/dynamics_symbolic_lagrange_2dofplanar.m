%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EXERCISE: WRITE THE LAGRANGE FORMULATION FOR A 2DOF PLANAR ROBOT ARM
%   fext are expressed in the base reference system
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
function dynamics_symbolic_lagrange_2dofplanar

syms q1 q2 qd1 qd2 l1 l2 a1 a2 m1 m2 I1 I2 real
q = [q1 q2];
qd = [qd1 qd2];
n=2;
J1 = [-a1*sin(q1) 0;
      a1*cos(q1) 0; 
       0    0];
J2 = [-l1*sin(q1)-a2*sin(q1+q2)  -a2*sin(q1+q2);
      l1*cos(q1)+ a2*cos(q1+q2)   a2*cos(q1+q2);
       0    0];
M = m1*J1'*J1+m2*J2'*J2 + [I1 0; 0 0] + [I2 I2; I2 I2];
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








