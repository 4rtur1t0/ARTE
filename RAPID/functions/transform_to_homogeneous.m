%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   T=transform_to_homogeneous(robtarget)
%
%   Given a robot target specified in RAPID as a position and quaternion.
%   Transform it to a position/orientation specified by a matrix T of position/orientation
%   
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche
%   Date: 05/05/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function T=transform_to_homogeneous(robtarget)

%position, do not transform to meters
pos = robtarget(1:3); 
Q = robtarget(4:7);

T = quaternion2T(Q);
T(1:3,4)=pos';