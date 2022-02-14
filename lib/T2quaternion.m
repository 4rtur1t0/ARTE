%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = T2quaternion(T)
%   Compute a quaternion Q from a rotation matrix T.
%
%   This implementation has been copied from The Robotics Toolbox for
%   Matlab of Peter Corke:
%   https://github.com/petercorke/spatial-math
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
function Q = T2quaternion_shepherd(T)

R = T(1:3, 1:3);

% modified version of sign() function as per the paper
%  sign(x) = 1 if x>=0
function s = sign(x) 
   if x >= 0
         s = 1;
   else
         s = -1;
   end
end
            
s = sqrt(trace(R)+1)/2.0;
kx = R(3,2) - R(2,3);   % Oz - Ay
ky = R(1,3) - R(3,1);   % Ax - Nz
kz = R(2,1) - R(1,2);   % Ny - Ox            
    
                
% equation (7)
[~,k] = max(diag(R));
switch k
    case 1  % Nx dominates
        kx1 = R(1,1) - R(2,2) - R(3,3) + 1; % Nx - Oy - Az + 1
        ky1 = R(2,1) + R(1,2);          % Ny + Ox
        kz1 = R(3,1) + R(1,3);          % Nz + Ax
        sgn = sign(kx);
    case 2  % Oy dominates
        kx1 = R(2,1) + R(1,2);          % Ny + Ox
        ky1 = R(2,2) - R(1,1) - R(3,3) + 1; % Oy - Nx - Az + 1
        kz1 = R(3,2) + R(2,3);          % Oz + Ay
        sgn = sign(ky);
    case 3 % Az dominates
        kx1 = R(3,1) + R(1,3);          % Nz + Ax
        ky1 = R(3,2) + R(2,3);          % Oz + Ay
        kz1 = R(3,3) - R(1,1) - R(2,2) + 1; % Az - Nx - Oy + 1
        add = (kz >= 0);
        sgn = sign(kz);
end

% equation (8)
kx = kx + sgn*kx1;
ky = ky + sgn*ky1;
kz = kz + sgn*kz1;

nm = norm([kx ky kz]);
if nm == 0
    % handle special case of null quaternion
    s = 1;
    v = [0 0 0];
else
    v = [kx ky kz] * sqrt(1 - s^2) / nm;  % equation (10)
end

Q=[s v];

end

