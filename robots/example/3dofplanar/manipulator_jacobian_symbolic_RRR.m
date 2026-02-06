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
function manipulator_jacobian_symbolic

syms q1 q2 q3 L1 L2 L3 real

px = L1*cos(q1)+L2*cos(q1+q2) + L3*cos(q1+q2+q3);
py = L1*sin(q1)+L2*sin(q1+q2) + L3*sin(q1+q2+q3);
pz = 0
phi = q1+q2+q3;

Jv = [diff(px, q1) diff(px, q2) diff(px, q3);
     diff(py, q1) diff(py, q2) diff(py, q3);
     0 0 0];
 
 Jw = [0 0 0;
       0 0 0;
       1 1 1];
   
 Jv
 Jw
 J = [Jv; Jw]
 Jt = J'
 
 tau = J'*[1 1 1 1 1 0]';
 
 tau
 a=tau(1)-tau(2)
 
 



