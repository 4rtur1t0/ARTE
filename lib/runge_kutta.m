%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [tout, yout]=runge_kutta(f, y0, timespan, timestep)
%
%   An implementation of the Runge?Kutta "RK4" or classical Runge?Kutta method. 
%   with:
%   f: function that returns dy/dt = f(t, y). A "pointer" to the function to
%       be integrated.
%   y0: initial value of y.
%   timespan: timespan is a vector [t0 tfinal], with t0 being the initial time
%       and tfinal the final time in the integration.
%   timestep: the fixed step size in the integration.
%
%   See also: arte/exercises/simulation/solution/runge_kutta_exercise.m for
%   a demo on how to use the runge_kutta function. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
function [tout, yout]=runge_kutta(f, y0, timespan, timestep)
%timestep for the integration
h=timestep;
t0 = timespan(1);
tfinal = timespan(2);

y = y0;
yout = y;                                         
tout=t0:h:tfinal;
for t=t0:h:tfinal-h,                             
    k1 = f(t, y);
    k2 = f(t + 0.5*h, y + 0.5*h*k1);
    k3 = f(t + 0.5*h, y + 0.5*h*k2);
    k4 = f(t + h, y + k3*h);

    y = y + h*(k1+2*k2+2*k3+k4)/6; 
    yout = [yout y];
end

