%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = T2quaternion2(T)
%   Returns the quaternion corresponding to an homogeneous transformation 
%   matrix T. Only the 3x3 rotation matrix in T is used.
%
%   See also QPROD, QUATERNION2T.
% The method implemented here was extracted from:
% Accurate Computation of Quaternions from Rotation Matrices. 
% Soheil Sarabandi and Federico Thomas
%  http://www.iri.upc.edu/files/scidoc/2068-Accurate-Computation-of-Quaternions-from-Rotation-Matrices.pdf
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
function Q = T2quaternion2(T)
% use only the orientation from T
R = T(1:3, 1:3);

if R(1,1) + R(2,2) + R(3,3) > 0
    Q(1) = 0.5*sqrt(1+R(1,1) + R(2,2) + R(3,3));
else
    num = (R(3,2)-R(2,3))^2 + (R(1,3)-R(3,1))^2 + (R(2,1)-R(1,2))^2;
    den = 3 - R(1,1) - R(2,2) - R(3,3);
    Q(1) = 0.5*sqrt(num/den);
end

if R(1,1) - R(2,2) - R(3,3) > 0
    Q(2) = 0.5*sqrt(1 + R(1,1) - R(2,2) - R(3,3));
else
    num = (R(3,2)-R(2,3))^2 + (R(1,3)+R(3,1))^2 + (R(2,1)+R(1,2))^2;
    den = 3 - R(1,1) + R(2,2) + R(3,3);
    Q(2) = 0.5*sqrt(num/den);
end

if -R(1,1) + R(2,2) - R(3,3) > 0
    Q(3) = 0.5*sqrt(1 - R(1,1) + R(2,2) - R(3,3));
else
    num = (R(3,2)+R(2,3))^2 + (R(1,3)-R(3,1))^2 + (R(2,1)+R(1,2))^2;
    den = 3 + R(1,1) - R(2,2) + R(3,3);
    Q(3) = 0.5*sqrt(num/den);
end

if -R(1,1) - R(2,2) + R(3,3) > 0
    Q(4) = 0.5*sqrt(1 - R(1,1) - R(2,2) + R(3,3));
else
    num = (R(3,2)+R(2,3))^2 + (R(1,3)+R(3,1))^2 + (R(2,1)-R(1,2))^2;
    den = 3 + R(1,1) + R(2,2) + R(3,3);
    Q(4) = 0.5*sqrt(num/den);
end




    