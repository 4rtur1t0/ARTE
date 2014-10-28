% THE DEMO PRESENTS AN ERROR PROPAGATION USING A MONTE-CARLO METHOD
% The error in the positioning of the joints is propagated to an error
% in the position of the end effector

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

close all

M=500; %number of particles

%load arm parameters
robot =  load_robot('example', 'scara')

%standard deviation in each of the three joints
sigmaq1=0.017;%rad
sigmaq2=0.017;%rad
sigmaq3=0.01;% m


%find errors around this pose
q=[pi/2 -pi/2 0 0];

puntos=[];
for i=1:M,
    qi = q + [normrnd(0, sigmaq1) normrnd(0, sigmaq2) normrnd(0, sigmaq3) 0];
    T=directkinematic(robot, qi);
    puntos=[puntos; T(1,4) T(2,4) T(3,4)];
end


adjust_view(robot)
drawrobot3d(robot,q), hold on
plot3(puntos(:,1),puntos(:,2), puntos(:,3),'r.')