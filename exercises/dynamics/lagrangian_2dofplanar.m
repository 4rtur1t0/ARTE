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
function lagrangian_2dofplanar

syms q1 q2 qd1 qd2 L1 L2 a1 a2 m1 m2 real

J1 = [-a1*sin(q1) 0;
      -a1*cos(q1) 0; 
       0    0];
J2 = [-L1*sin(q1)-a2*sin(q1+q2) -a2*sin(q1+q2);
      L1*cos(q1)+a2*cos(q1+q2)   a2*cos(q1+q2);
       0    0];
Kv = m1*J1'*J1+m2*J2'*J2;
Kv = simplify(Kv)







