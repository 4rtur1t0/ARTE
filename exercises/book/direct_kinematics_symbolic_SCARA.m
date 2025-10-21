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
function direct_kinematics_symbolic_SCARA
% link lengths
format short
digits(2)
syms q1 q2 q3 q4 L1 L2 L3

%q=[1 0 pi/2 0.5 pi/2 0.5];
%q=[1 pi/2 -pi/2 0.5 0 0.5];

% matrices DH
A01 = dh_sym(q1, L1, L2, 0)
A12 = dh_sym(q2, 0, L3, pi)
A23 = dh_sym(0, q3, 0, 0)
A34 = dh_sym(q4, 0, 0, 0)


%sympref('FloatingPointOutput',true);

T = A01*A12*A23*A34;
T = simplify(T);
T = vpa(T)




 
 




function A = dh_sym(theta, d, a, alpha)
syms q1 q2 q3 q4 L1 L2 L3
% avoid almost zero elements in cos(alpha) and sin(alpha)
ca = cos(alpha);
sa = sin(alpha);
if abs(ca) < 1e-6
    ca = 0;
end
if abs(sa) < 1e-6
    sa = 0;
end


A=[cos(theta)  -ca*sin(theta)   sa*sin(theta)   a*cos(theta);
   sin(theta)   ca*cos(theta)  -sa*cos(theta)   a*sin(theta);
            0         sa             ca             d;
            0         0               0             1];
        

        
