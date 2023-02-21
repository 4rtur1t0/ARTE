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
function gimbal_euler
% A gimbal NASA APOLLO MISSIONS!

syms thp thr thy

% matrices DHs
R01 = R_sym(thp, 'x');
R12 = R_sym(thr,'y');

R02 = R01*R12;
R02 = simplify(R02)

%thp = 0.1
%thp = 0.2

J = [1 0 0]';
J = [J R01(:,2) R02(:,3)]

detJ = simplify(det(J))

R01 = eval(R01)
R02 = eval(R02)

figure, hold, grid
draw_axes(eye(4), 'X0', 'Y0', 'Z0', 1);
draw_axes(R01, 'X1', 'Y1', 'Z1', 1);
draw_axes(R02, 'X2', 'Y2', 'Z2', 1);

 


function R = R_sym(var, axis)
syms thp thr thy

if axis == 'x'
    R = [1 0 0;
         0 cos(var) -sin(var);
         0  sin(var) cos(var)];
elseif axis == 'y'
    R = [cos(var) 0 sin(var);
            0     1     0;
        -sin(var) 0 cos(var)];
elseif axis == 'z'
    R = [cos(var) -sin(var) 0;
         sin(var)  cos(var) 0;
            0         0     1];
else
    'UNKNOWN AXIS'
end

        
