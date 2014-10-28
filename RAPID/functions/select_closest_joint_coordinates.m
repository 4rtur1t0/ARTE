%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q=SELECT_CLOSEST_JOINT_COORDINATES(ROBOT, Qinv, Qcurrent)
% Given a matrix Qinv, where each column corresponds to a different
% solution to the inverse kinematic problem. The function returns
% Q that are closest to Qcurrent
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
function q=select_closest_joint_coordinates(qinv, q_current)

a = qinv-repmat(q_current(:), 1, size(qinv,2));
a=sum(abs(a));
[val,i]=min(a);

q=qinv(:,i); 

