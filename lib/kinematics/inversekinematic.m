%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   INVERSEKINEMATIC	Inverse kinematic for serial robots.
%
% 	q = INVERSEKINEMATIC(robot, T) 
%   returns the vector q of joint values according to the transformation matrix
%   of the end effector T.
%
%   INVERSEKINEMATIC calls a pre-defined function specified by ROBOT.inversekinematic_fn that
%   should be implemented in the inversekinematic directory.
%
%	See also DIRECTKINEMATIC.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   26/04/2012
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
function q = inversekinematic(robot, T)

if robot.debug
    fprintf('\nComputing inverse kinematics for the %s robot\n', robot.name);
    fprintf('\nCall to: inversekinematic/%s\n', robot.inversekinematic_fn);
end
%Call specific inversekinematic function
q = eval(robot.inversekinematic_fn);
