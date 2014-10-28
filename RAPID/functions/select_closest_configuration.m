%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q=SELECT_CLOSEST_CONFIGURATION(ROBOT, Qinv, CONF)
% Returns the joint coordinates Q that are closest with the axes configuration
% vector CONF, given a set of solutions of the inverse kinematic problem Qinv.
% For 6DOF or less manipulators, the variable CONF={CF1, CF4, CF6, CFX}
% specifies univoquely only one of the solutions. The first axis has
% priority over the rest.
%
% See also:
%   COMPUTE_CONFIGURATION, SELECT_CONFIGURATION , GET_CONF_DATA
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function q=select_closest_configuration(robot, qinv, conf)

q=qinv(:,1); 

distance=64;

for i=1:size(qinv,2),    
    confi=compute_configuration(robot, qinv(:,i));
    
    d = abs(conf(1)-confi(1)) + abs(conf(2)-confi(2)) + abs(conf(3)-confi(3));
    
    if d < distance
       q =  qinv(:,i);
       distance=d;       
    end
end

if d ~= 0 
    disp('WARNING: RAPID/select_closest_configuration: No solutions complies with the specified configuration ');
end
