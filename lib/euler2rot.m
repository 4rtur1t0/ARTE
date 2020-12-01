%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   R = euler2rot(abc, convention)
%   Returns the Rotation angle for three euler angles abc=[alpha beta gamma].
%
%   The convention specifies the order and axes of the rotations. For
%   example, for the 'XYZ' convention, we have
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
function [R] = euler2rot(abc, convention)

if convention =='XYZ'
    ax = abc(1);    
    ay = abc(2);
    az = abc(3);

    Rx = Rot(ax, 'x');
    Ry = Rot(ay, 'y');
    Rz = Rot(az, 'z');

    R = Rx*Ry*Rz;
else
    'Unknown convention'
end