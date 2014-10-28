%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q=SELECT_CONFIGURATION(ROBOT, Qinv, CONF)
% Returns the joint coordinates Q that comply with the axes configuration
% vector CONF, given a set of solutions of the inverse kinematic problem Qinv.
% For 6DOF or less manipulators, the variable CONF={CF1, CF4, CF6, CFX}
% specifies univoquely only one of the solutions.
%
% See also:
%   COMPUTE_CONFIGURATION, GET_CONF_DATA
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
function q=select_configuration(robot, qinv, conf)

q=qinv(:,1); %zeros(robot.DOF,1);

for i=1:size(qinv,2),
    confi=compute_configuration(robot, qinv(:,i))
    if isequal(conf(1:4), confi(1:4))
       %if the same configuration is found, store the joint values and
       %return
       q = qinv(:,i);
       return;
    end
end

disp('ERROR: RAPID/select_configuration: No solutions complies with the exact specified configuration ');

disp('WARNING: Selecting now the closest configuration');

q=select_closest_configuration(robot, qinv, conf);


