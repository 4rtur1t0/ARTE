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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  draw equipment such as tables, conveyor velts... etc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(robot, 'equipment')
        if robot.equipment.graphical.has_graphics
            draw_link(robot.equipment, 1, robot.equipment.T0);
        end
        if robot.equipment.graphical.draw_axes
            draw_axes(robot.equipment.T0, sprintf('X_{env%d}',0), sprintf('Y_{env%d}',0), sprintf('Z_{env%d}',0));
        end      
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  draw equipment tools
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(robot, 'tool')
        if robot.tool.graphical.has_graphics
            T=directkinematic(robot,q);
            draw_link(robot.tool, 1, T);
        end
        if robot.equipment.graphical.draw_axes
            draw_axes(T, sprintf('X_{tool%d}',0), sprintf('Y_{tool%d}',0), sprintf('Z_{tool%d}',0));
        end      
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DRAW A PIECE GRABBED
% this draws a piece grabbed at the robots end, or at the last known position
% the variable robot.tool_activated=1 asumes that the piece has been grabbed by the end tool
% robot.tool_activated==0 maintains the last known location for the pieced
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(robot, 'piece')
    if robot.piece.graphical.has_graphics        
        if isfield(robot, 'tool')
            if robot.tool.piece_gripped==1
                %update the last known position and orientation of the robot
                %Trel is computed when the simulation_grip_piece function is
                %executed. Trel is the relative position and orientation of
                %the tool and the piece that assures that the piece is
                %picked at a constant and visually effective orientation.
                draw_link(robot.piece, 1, T07*(robot.tool.Trel));
            else
                draw_link(robot.piece, 1, robot.piece.T0);
            end
        end
    else
        draw_link(robot.piece, 1, robot.piece.T0);
    end
    if robot.piece.graphical.draw_axes
        draw_axes(robot.piece.T0, sprintf('X_{piece%d}',0), sprintf('Y_{piece%d}',0), sprintf('Z_{piece%d}',0));
    end
end
