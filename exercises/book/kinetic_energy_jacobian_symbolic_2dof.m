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
function kinetic_energy_jacobian_symbolic_2dof

syms L1 L2 real
syms I1 I2 real
syms q1 q2 real
syms qd1 qd2 real
syms m1 m2 real
Jvc2 = [-L1*sin(q1)-L2*sin(q1+q2)/2, -L2*sin(q1+q2)/2;
         L1*cos(q1)+L2*cos(q1+q2)/2,  L2*cos(q1+q2)/2;
         0, 0    ];
vc2 = Jvc2*[qd1 qd2]';
% vc2 = norm(vc2);
% I1 = (1/3)*m1*L1^2*qd1^2;
% K2 = m2*L2^2/3*qd2^2;
K1 = (1/2)*I1*qd1^2;
K2 = (1/2)*I2*qd2^2;
K21 = (1/2)*m2*vc2'*vc2;
K21 = simplify(K21);
qd2 = 0;
K21 = eval(K21);

K = K1 + K2 + K21;
K = simplify(K)

