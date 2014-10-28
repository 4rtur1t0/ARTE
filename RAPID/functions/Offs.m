%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Offs(robot, robtarget, deltaX, deltaY, deltaZ, gripper, Wobj)
%   Compute a point displaced (deltaX, deltaY, deltaZ) m from the specified
%   point robtarget. The displacement is performed in the Wobj reference
%   system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function robtarget = Offs(robtarget, deltaX, deltaY, deltaZ)

global configuration

fprintf('\nCall to Offs');

robtarget(1)=robtarget(1)+deltaX;
robtarget(2)=robtarget(2)+deltaY;
robtarget(3)=robtarget(3)+deltaZ;


