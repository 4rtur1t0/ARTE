%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = QROT(THETA, AXIS)
%   Returns a quaternion corresponding to a rotation of theta over axis u.
%   where u can be: 'i', 'j', or 'k' for rotations over the X, Y and Z axes
%   respectively.
%   
%   See also QPROD, T2QUATERNION, QUATERNION2T.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. email:
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
function Q = qRot(theta, axis)

switch axis
    case 'i'
        Q = [cos(theta/2) sin(theta/2) 0 0];
    case 'j'
        Q = [cos(theta/2) 0 sin(theta/2) 0];
    case 'k'
        Q = [cos(theta/2) 0 0 sin(theta/2)];
    otherwise
        disp('Please: select i, j or k')
        Q = [1 0 0 0];
end