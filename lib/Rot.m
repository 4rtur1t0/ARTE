%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q=Rot(theta, axis)
%   Compute a rotation matrix along the specified axis.
%
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
function R = Rot(theta, axis)

switch lower(axis)
    case {'x'}
        R = [1        0          0;
             0   cos(theta) -sin(theta);
             0   sin(theta)  cos(theta)];
    case {'y'}
        R = [ cos(theta) 0  sin(theta);
               0         1     0; 
             -sin(theta) 0  cos(theta)];   
   case {'z'}
        R = [cos(theta) -sin(theta) 0;
             sin(theta)  cos(theta) 0;
                  0          0      1];     
   otherwise
            disp('Unknown axis. Please specify X, Y or Z.')
end

