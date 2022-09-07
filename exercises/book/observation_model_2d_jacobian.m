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
function observation_model_2d_jacobian()


syms xi yi thi xj yj thj xij yij thij real


% matrices DH
Ti = T_sym(xi, yi, thi);
Tj = T_sym(xj, yj, thj);
Tij = T_sym(xij, yij, thij);

Tr = inv(Ti)*Tj
Tr = simplify(Tr)

Te = inv(Tij)*(inv(Ti)*Tj)
Te = simplify(Te)

Te(1,4)


eij = R_sT(thij)*(R_sT(thi)*([xj yj]'-[xi yi]')-[xij yij]')
eij = simplify(eij)
eij(1)
%ep2 = thj-thi-thij
simplify(eij(1)-Te(1,4))
simplify(eij(2)-Te(2,4))

% Aij = zeros(4, 4);
Aija = simplify(-R_sT(thij)*R_sT(thi))
%Aijb(4, 1:3) = [0 0 0]
Aijb = simplify(R_sT(thij)*dR_dthT(thi)*([xj yj]'-[xi yi]'))

Bij = simplify(R_sT(thij)*R_sT(thi))




function T = T_sym(x, y, th)

syms xi yi thi xj yj thj xij yij thij

T=[cos(th)  -sin(th)   0   x;
   sin(th)   cos(th)   0   y;
       0        0      1   0;
       0        0      0   1];

function T = R_s(th)

syms thi thj thij

T=[cos(th)  -sin(th);
   sin(th)   cos(th)];
   
function T = R_sT(th)
syms thi thj thij

T=[cos(th)    sin(th);
   -sin(th)   cos(th)];

function T = dR_dth(th)
syms thi thj thij

T=[-sin(th)    -cos(th);
   cos(th)   -sin(th) ];  

function T = dR_dthT(th)
syms thi thj thij

T=[-sin(th)    cos(th);
   -cos(th)   -sin(th)];  
   
   