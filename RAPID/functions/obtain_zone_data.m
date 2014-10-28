%given a tag specifying the zone data. Return a radius in meters
% for example, given z100, return radius=0.1 m

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
function radius = obtain_zone_data(zonedata)

if strncmp(zonedata, 'fine',4) %case fine
   radius = 0;
else 
    [tag,remain] = strtok(zonedata, 'z');
    radius = eval(tag)/1000;
end

