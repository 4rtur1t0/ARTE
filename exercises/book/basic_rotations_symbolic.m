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
function basic_rotations_symbolic
% euler angles

syms a b c



% matrices DH
Ra = R_sym(a, 'z');
Rb = R_sym(b, 'x');

R = Ra*Rb
R = simplify(R)

%degenerate case 1
a = pi/2;
b=pi/2;
R = eval(R)
R(abs(Rb)<0.001)=0;
R = simplify(R)


function R = R_sym(var, axis)
syms a b c

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

        

        
