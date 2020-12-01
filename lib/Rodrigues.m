%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   R=Rodrigues(k)
%
%   Given a Rodrigues vector k, compute its corresponding rotation matrix R.
%   
%   Caution: the Rodrigues vector is a not unitary norm representation.
%   That is, if u is a unit vector, then:
%                   k = theta*u
%   where theta is the rotation angle. This last representation corresponds
%   to the typical Euler angle-axis representation of rotation with the
%   pair:
%                   (theta, u)
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. email:
%   arturo.gil@umh.es date:   02/10/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function R = Rodrigues(k)
th = norm(k);
k = k/th;
kx = k(1);
ky = k(2);
kz = k(3);
K = [0  -kz  ky;
     kz  0  -kx;
    -ky  kx  0];
R=eye(3) + sin(th)*K + (1-cos(th))*K*K;