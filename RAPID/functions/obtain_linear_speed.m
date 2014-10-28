% compute linear end effector's speed according to speeddata variable
% vmax means maximum speed specified by the manufacturer

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
function vel=obtain_linear_speed(robot, speeddata)

if strncmp(speeddata, 'vmax',4);
    vel=robot.linear_velmax;
else
    [tag,remain] = strtok(speeddata, 'v');
    vel = robot.linear_velmax*eval(tag)/1000;
end