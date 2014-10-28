%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  [T Q QD]= FORWARDDYNAMIC(ROBOT, TIME_END, Q0, Qd0, TAU, torqfun, varargin)
%
%  Compute forwarddynamics for the robot ROBOT for a period of TIME_END
%  seconds. The initial state is defined by joint position Q0 and joint
%  velocity Qd0. A constant vector of torques TAU is specified.
%
%  Returns a time vector T, position Q and velocity QD when applying a
%  torque TAU during a time period time_end
%
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
%   email: arturo.gil@umh.es date:   26/04/2012
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
function [t, q, qd] = forwarddynamic(robot, time_end, q0, qd0, tau, g, torqfun, varargin)

	n = robot.DOF;

    % concatenate q and qd into the initial state vector
    q0 = [q0(:); qd0(:)];
		
	[t,y] = ode45(@fdyn_private, [0 time_end], q0, [], tau, g, robot, torqfun, varargin{:});

    q = y(:,1:n)';
	qd = y(:,n+1:2*n)';

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   FDYN_PRIVATE  private function called by FORWARDDYNAMIC
%
%	XDD = FDYN_PRIVATE(T, X, TAU, ROBOT, TORQUEFUN)
%
%   FORWARDDYNAMIC calls this function to evaluate the velocity and
%   acceleration. 
%   TIME is the current time. 
%   X = [Q QD] is the state vector
%   TAU is a vector of contant torques applied at each joint 
% 
%   TORQUEFUN is the string name of the function to compute joint torques and called as
%
%       TAU = TORQUEFUN(T, X)
%
% if not given zero joint torques are assumed.
%
% The function returns XDD = [QD QDD].
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = fdyn_private(time, x, tau, g, robot, torqfun, varargin)
%time
	n = robot.DOF;

	q = x(1:n)';
	qd = x(n+1:2*n)';
    
	
	qdd = accel(robot, x(1:n,1), x(n+1:2*n,1), tau, g);
	xd = [x(n+1:2*n,1); qdd];
end
