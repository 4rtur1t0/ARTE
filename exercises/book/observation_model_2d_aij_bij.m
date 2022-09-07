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
function observation_model_2d_aij_bij()


syms xi yi thi xj yj thj xij yij thij real

Aija = simplify(-RT(thij)*RT(thi))
Aijb = simplify(RT(thij)*dRdthT(thi)*([xj yj]'-[xi yi]'))
Bij = simplify(RT(thij)*RT(thi))

Aij = 


function R = R(th)
syms thi thj thij

R =[cos(th)  -sin(th);
   sin(th)   cos(th)];
   
function R = RT(th)
syms thi thj thij

R=[cos(th)    sin(th);
   -sin(th)   cos(th)];

function R = dRdth(th)
syms thi thj thij

R=[-sin(th)  -cos(th);
   cos(th)   -sin(th) ];  

function R = dRdthT(th)
syms thi thj thij

R=[-sin(th)    cos(th);
   -cos(th)   -sin(th)];  
   
   