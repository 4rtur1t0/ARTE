%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This function is called from the simulate_2dofrobot simulink model.
%   This function receives as inputs the initial and final desired
%   joint positions q, and the desired initial and final desired joint
%   speeds qd. Also, the total time spent in the movement is specified from
%   the simulink model. 
%   
%   When the current time is zero, the values of the coefficients k of qd_t
%   qd_t and qdd_t are computed. Next, at each call of the function, the
%   current values of q_t, qd_t and qdd_t are computed.
%
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 08/03/2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function [out]=call_third_order_planner(u)

%global k
q=u(1:2);
qd=u(3:4);
total_time=u(5); % total time, [0, final time]
current_time=u(6); %current time, yes, compute the values of the splines for the current simulation time


ttime=[0 total_time];
%   third order spline
% in the first iteration, compute
A=[1 ttime(1) ttime(1)^2 ttime(1)^3;
    1 ttime(2) ttime(2)^2 ttime(2)^3;
    0   1  2*ttime(1) 3*ttime(1)^2;
    0   1  2*ttime(2) 3*ttime(2)^2];
k=inv(A)*[q(1) q(2) qd(1) qd(2)]';


t=current_time;
q_t = k(1) + k(2)*t + k(3)*t^2 + k(4)*t^3;
qd_t= k(2) + 2*k(3)*t + 3*k(4)*t^2;
qdd_t=2*k(3) + 6*k(4)*t;

%output, joint position speed and acceleration
out=[q_t, qd_t, qdd_t];
