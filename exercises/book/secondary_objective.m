% The null space of a 3DOF robot when considered redundant.
% if the task m=(vx, vy), then the robot is redundant in lambda=3
%
% Copyright (C) 2016, by Arturo Gil Aparicio
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
function secondary_objective
%lateral function

syms qi qimax qimin


%w = 1/((qimax - qi)*(qi - qimin));
%dw = (qimax-qimin)diff(w, qi)
w = 1/(-qi^2+qi*(qimax + qimin)-qimax*qimin);
dw = (qimax-qimin)*diff(w, qi)

        
