%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% index = find_first_in_zone_data(robot, q, T, radius)
%
% Returns the index in the vector q that first goes into a sphere of
% the specified radius around the target specified by T
% if radius==0, the last index is returned
% q is a matrix, where each column stores robot.DOF joint coordinates at a
% particular time.
%
% Author:  Arturo Gil. Universidad Miguel Hernández de Elche
% Date: 07/04/2012
%
%   See also: test_joints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function index = find_first_in_zone_data(robot, q, T, radius)

if radius==0
    index=size(q,2);
    return;
end

final = T(1:3,4);

for index=1:size(q,2),
   Ti = directkinematic(robot, q(:,index));
   current = Ti(1:3,4);  
   distance = ((final-current)'*(final-current));
   if distance <= radius^2
       %return current index
       return;
   end
end