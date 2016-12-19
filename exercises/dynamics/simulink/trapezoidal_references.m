%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  TRAPEZOIDAL_REFERENCES
%  Return speed and acceleration when the robot makes a trapezoidal speed
%  Obtain q when the robot makes a linear trajectory of the end effector 
%  in cartesian space
%
%  See also INVERSEKINEMATIC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%Computes a trapezoidal speed profile for every joint given maximum
%permitted accelerations and maximum joint speeds
function [out]=trapezoidal_references(u)

global robot

%delta_time=0.01;
time = u(1);
total_time = u(2);

%build time vector: twice acceleration time plus time at constant speed
%time = 0:delta_time:total_time;

maximum_speeds = robot.velmax;
maximum_accels = robot.accelmax;

input_speeds=[];
input_accelerations=[];

for j=1:length(maximum_speeds), 
    %vel_row=[];
    %acc_row=[];
    %for i=1:length(time), 
    [vel, acc] = compute_values(time, maximum_speeds(j), maximum_accels(j), total_time);
    %vel_row = [vel_row vel];
    %acc_row = [acc_row acc];        
    %end
    input_speeds = [input_speeds; vel];
    input_accelerations = [input_accelerations; acc];    
end
out = [input_speeds; input_accelerations];




%returns the values of velocity and speed corresponding to a given time
function [vel, acc]=compute_values(time_i, vel_max, acc_max, total_time)

tacc = vel_max/acc_max;
tdec = total_time-tacc;

if time_i < tacc
    vel = time_i.*acc_max;
    acc = acc_max;
    return;
elseif (time_i >= tacc) & (time_i < tdec)
    vel = vel_max;
    acc = 0;
    return;
else % time_i> tdec
    vel = vel_max-(time_i-tdec)*acc_max;
    acc = -acc_max;    
end