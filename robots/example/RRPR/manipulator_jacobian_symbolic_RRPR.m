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

syms q1 q2 q3 q4 phi L1 L2 real

px = cos(q1)*(L2*cos(q2+q4)+q3*cos(q2))
py = sin(q1)*(L2*cos(q2+q4)+q3*cos(q2))
pz = L1 + L2*sin(q2+q4) + q3*sin(q2)

Jv = [diff(px, q1) diff(px, q2) diff(px, q3) diff(px, q4);
     diff(py, q1) diff(py, q2) diff(py, q3) diff(py, q4);
     diff(pz, q1) diff(pz, q2) diff(pz, q3) diff(pz, q4)];
 
 Jw = [0 sin(q1) 0 sin(q1);
       0 -cos(q1) 0 -cos(q1);
       1 0 0 0];
   
 Jv
 Jw
 J = [Jv; Jw]
 JT = J'
 fn = [0 0 1 0 0 0]';
 tau = J'*fn;
 L2 = 0.5;
 q1 = 0;
 q2 = asin(0.5);
 q3 = 6*tan(q2);
 q4 = acos(0) - q2;
 
 J = eval(J);
 
 tau = J'*fn;
 
 



