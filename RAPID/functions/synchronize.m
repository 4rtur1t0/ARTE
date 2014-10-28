%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	[velo2, tmax]= SINCHRONIZE(qini, qfinal, velocity) Finds a mean speed and the required 
%   time to perform a movement between the joint coordinates qini and qfinal. 
%   If the speed of each joint is different, the maximum time to perform the movement
%   by the slower joint is taken as a basis.
%   
%   Inputs:
%		Qini: initial position in joint coordinates.
%		Qfinal: final position in joint coordinates.
%		Velocity: stores the maximum velocity of each joint.
%   Outputs:
%        velo2: new maximum speed for each joint.
%        tmax: time needed to perform the movement.
%
%	See also: MOVEJ, COMPUTE_JOINT_TRAJECTORY_INDEP
%   
%   Author: Arturo Gil
%   Date:   29/03/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function [actual_speed, maxtime]=synchronize(qini, qfinal, speed, accel)

tacel = speed./accel;

tcte = (abs(qfinal(:)-qini(:))-accel(:).*tacel.^2)./speed(:);

time_total = tcte + 2*tacel;

maxtime=max(time_total);

actual_speed = (qfinal(:)-qini(:)-accel(:).*tacel.^2)/maxtime(:);