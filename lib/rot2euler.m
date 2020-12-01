%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [alpha, beta, gamma] = rot2euler(R, convention)
%   Returns the Euler angles alpha, beta and gamma that yield a given matrix
%   R. The convention specifies the order and axes of rotations.
%   Currently, only the XYZ convention is supported. 
%
%   In particular, the function computes the angles alpha, beta and gamma that
%   allow to compute R as.
%
%   R = Rot(alpha,'x')*Rot(beta,'y')*Rot(gamma,'z')
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. email:
%   arturo.gil@umh.es date:   11/11/2020
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
function [alpha, beta, gamma] = rot2euler(R, convention)


if convention=='XYZ'
    [alpha, beta, gamma]=conventionXYZ(R);
else
    'Unknown convention. Only XYZ is supported'
end



function [alpha, beta, gamma] = conventionXYZ(R)
%'R(1,3)=sen(beta)=1??'
if abs(R(1,3)) == 1
    % degenerate case in which sen(beta)=+-1 and cos(beta)=0
    alpha = 0; % arbitrarily set alpha to zero
    beta = asin(R(1,3));
    gamma = atan2(R(2,2), R(2,1));    
else
    % standard way to compute alpha beta and gamma
    alpha = -atan2(R(2,3), R(3,3));
    gamma = -atan2(R(1,2), R(1,1));
    beta = atan2(cos(alpha)*R(1,3), R(3,3));
end

