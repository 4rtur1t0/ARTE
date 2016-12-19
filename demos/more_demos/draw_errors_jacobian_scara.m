% PROPAGATE A GAUSSIAN ERROR DISTRIBUTION OF EACH JOINT TO AN ERROR IN
% END EFFECTORS' POSITION
% AN ELLIPSE IS DRAWN TO REPRESENT THE ERROR IN POSITION

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

%load arm parameters
robot= load_robot('example','scara')

%find errors around pose
q=[pi/4 3*pi/4 0 0];

%Matriz de errores en las articulaciones
% sigmaq1=sigmaq2=0.0017 rad, sigmaq3=0.01 m.
sigmaq1=0.017;%rad, 0.01 grados
sigmaq2=0.017;%rad
sigmaq3=0.01;% m
Rq=[sigmaq1^2 0 0;
    0 sigmaq2^2 0;
    0 0 sigmaq3^2]

teta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);

Jq = eval(robot.J)


Rp=Jq*Rq*Jq'

T=directkinematic(robot, q);

drawrobot3d(robot,q), hold on
draw_ellipse([T(1,4),T(2,4)], Rp, 'r')