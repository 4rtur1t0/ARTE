%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  [ttotal, traj_type, qt, qdt, qddt, time]=trapezoidal_isochronous(q0, qf, speed_max, accel_max, delta_time)
%
%   Compute a trapezoidal profile function in position, speed and
%   acceleration given a set of maximums speeds and accelerations for each
%   joint. The algorithm tries to perform the whole movement in minimum
%   time, achieving, at least, one of the joints, the maximum speed and or
%   acceleration.
% 
%   Valid for each of the joints in a robot or for Cartesian or Euler
%   interpolation.
%   
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
%   email: arturo.gil@umh.es date:   08/10/2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%
% Copyright (C) 2019, by Arturo Gil Aparicio
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
function [qt, qdt, qddt, time]= trapezoidal_isochronous(q0, qf, speed_max, accel_max, delta_time)
ttotal = [];
qt = [];
qdt = [];
qddt = [];
% Plan a path using omega, alpha
% returns the total time needed for each joint
for i=1:length(q0)
    [ttotal_i, traj_type, qt_, qdt_, qddt_, time]=trapezoidal(q0(i), qf(i), speed_max(i), accel_max(i), delta_time, 'plan_time');
    ttotal = [ttotal ttotal_i];
end
% find the max time 
t_coord = max(ttotal);
%now, plan all the joints using the time for a isochronous movement.
for i=1:length(q0)
    [qt_, qdt_, qddt_, time]=caseB(q0(i), qf(i), t_coord, alpha_max(i), delta_time, 'plan_speed');
    qt = [qt; qt_];
    qdt = [qdt; qdt_];
    qddt = [qddt; qddt_];
end









