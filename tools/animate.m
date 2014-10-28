%   ANIMATE(ROBOT, Q) 
%   Animate the robot in 3D when performing a trajectory.
%   The current robot parameters are stored in ROBOT and Q is a matrix
%   where each column stores the robot's joint coordinates.
%   Q = [q1, q2, q3,... qn]
%
%   ANIMATE iterates through the vector and draws the robot at each joint
%   coordinates.
%
%	See also DRAWROBOT3D.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   05/01/2012

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
function animate(robot, q, line)
global configuration 

h=figure(configuration.figure.robot);, hold on,
%adjust 3D view as desired
%adjust_view(robot)
v=zeros(3,3);
if exist('line', 'var')
    T1=directkinematic(robot, q(:,1));
    T2=directkinematic(robot, q(:,end));

    v=[T1(1:3,4) T2(1:3,4)];
end

%get adjusted view
[az,el] = view;
for j=1:size(q, 2);
    clf(h);
    qj=q(:,j);  
    view(az,el);
    %draw robot in 3D 
    %robot=drawrobot3d_simulation(robot, qj);  

    %robot=
    drawrobot3d(robot, qj);  

    
    %plot3(path(1,:),path(2,:),path(3,:),'k', 'LineWidth', 3);
    plot3(v(1,:),v(2,:),v(3,:),'k', 'LineWidth', 3);
    
    %pause to get a nice view
    pause(configuration.time_delay);   
end