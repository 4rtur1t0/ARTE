%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = T2quaternion(T)
%   Returns the quaternion corresponding to an homogeneous transformation 
%   matrix T. Only the 3x3 rotation matrix in T is used.
%
%   See also QPROD, QUATERNION2T.
%
%   Author: Arturo Gil. Universidad Miguel Hern�ndez de Elche. email:
%   arturo.gil@umh.es date:   21/04/2012
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
function Q = T2quaternion(T)
% use only the orientation
R = T(1:3, 1:3);

Q = zeros(1,4);
Q(1) = sqrt(trace(R))/2;
Q(1) = real(Q(1));

Lx = R(3,2) - R(2,3);	
Ly = R(1,3) - R(3,1);	
Lz = R(2,1) - R(1,2);	

if (R(1,1) >= R(2,2)) && (R(1,1) >= R(3,3))
    Lx1 = R(1,1) - R(2,2) - R(3,3) + 1;	
    Ly1 = R(2,1) + R(1,2);			
    Lz1 = R(3,1) + R(1,3);
elseif (R(2,2) >= R(3,3))
    Lx1 = R(2,1) + R(1,2);			
    Ly1 = R(2,2) - R(1,1) - R(3,3) + 1;	
    Lz1 = R(3,2) + R(2,3);		
else
    Lx1 = R(3,1) + R(1,3);			
    Ly1 = R(3,2) + R(2,3);			
    Lz1 = R(3,3) - R(1,1) - R(2,2) + 1;	
end

if (Lx >= 0) || (Ly >= 0) || (Lz >= 0)
    Lx = Lx + Lx1;
    Ly = Ly + Ly1;
    Lz = Lz + Lz1;
else
    Lx = Lx - Lx1;
    Ly = Ly - Ly1;
    Lz = Lz - Lz1;
end

if norm([Lx Ly Lz]) == 0
    Q = [1 0 0 0];
else
    s = sqrt(1-Q(1)^2)/norm([Lx Ly Lz]);
    Q(2:4) = s*[Lx Ly Lz];
end




    