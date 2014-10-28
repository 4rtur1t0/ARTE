%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DRAWROBOT3D_par(ROBOT, Q)	
% 3D drawing with DH reference systems for a parallel robot.
% The parallel robot is divided into different serial arms, drawn altogether
% 
% If the robot possesses 3D graphics, the function DRAW_LINK is called to represent
% the link in 3D. Otherwise a line connecting each consecutive reference
% system is plotted.
%
% See also DRAWROBOT3D, DENAVIT, DIRECTKINEMATIC, DRAW_LINK.
%
%   Author: Arturo Gil. Universidad Miguel Hern�ndez de Elche. 
%   email: arturo.gil@umh.es date:   05/02/2012
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
function drawrobot3d_par(robot, q, noclear)
%global configuration

%origin=[0 0 0];
endpoints=[];
%draw every arm considered in the parallel mechanism
k=1;
for i=1:robot.nserial,
    r=eval(sprintf('robot.robot%d',i));
    
    fi=q(k:(k+r.DOF-1));
    k=k+r.DOF;
    drawrobot3d(r,fi,0);
    
    T=eval(sprintf('directkinematic(robot.robot%d, fi)',i));
    destination=T(1:3,4);
    endpoints = [endpoints destination];   
       
end
%Add the first point
endpoints = [endpoints endpoints(:,1)];

%now, draw a line between the end points including the first
for i=1:robot.nserial,
    origin=endpoints(:,i);
    destination=endpoints(:,i+1);
    line([origin(1) destination(1)],[origin(2) destination(2)],[origin(3) destination(3)], 'Color',  'k' ,'LineWidth', 3 );     
end


