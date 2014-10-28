%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Torque = FRICTION(ROBOT, QD, j)
%
%  Computes torque based on viscous friction and Coulomb friction for the
%  joint j.
%
%  The torque is computed as:
%  tau = G^2*B*qd + abs(Tc)*abs(G);
% 
% The viscous friction parameter for each joint j must be stored in the
% variable robot.motors.Viscous, whereas the vector robot.motors.Coulomb 
% positive or negative parameters of the Coulomb friction
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
%   email: arturo.gil@umh.es date:   26/06/2012
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
function  torque = friction(robot, qd, j)

%torque due to viscous friction
%torque = robot.motors.G(j)^2*robot.motors.Viscous(j)*qd(j);
torque = robot.motors.G(j)*robot.motors.Viscous(j)*qd(j);

%Add torque due to Coulomb friction
if qd(j) > 0
    torque = torque + abs(robot.motors.G(j))*robot.motors.Coulomb(j,1);  
elseif qd(j) < 0
    torque = torque + abs(robot.motors.G(j))*robot.motors.Coulomb(j,2);
end



 


