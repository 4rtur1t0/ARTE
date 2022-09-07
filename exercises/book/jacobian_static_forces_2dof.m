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
function jacobian_static_forces_2dof
%syms q1 q2
L1 = 0.5;
L2 = 0.3;

q1 = pi/4;
q2 = 0;
 
% define the manipulator jacobian full
J = [-L2*sin(q1+q2)-L1*sin(q1) -L2*sin(q1+q2);
     L2*cos(q1+q2)+L1*cos(q1) L2*cos(q1+q2);
     0 0;
     0 0;
     0 0;
     1 1];

 
 fn = [1 1 1 1 1 1]';
 tau = J'*fn