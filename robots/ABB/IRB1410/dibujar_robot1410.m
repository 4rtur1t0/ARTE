%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DRAWROBOT3D(ROBOT, Q)	3D drawing with DH reference systems of the robot at the
% current joint coordinates Q. The robot parameters are stored in the
% variable ROBOT. Q = [q1 q2 ... qN] is a vector of joint values.
%
% If the robot possesses 3D graphics, the function DRAW_LINK is called to represent
% the link in 3D. Otherwise a line connecting each consecutive reference
% system is plotted.
%
% See also DENAVIT, DIRECTKINEMATIC, DRAW_LINK.
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
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
function drawrobot3d(robot, q, robot2, q2)
global configuration

%ligth;

%if q is not specified, then assume zero pose
if ~exist('q', 'var')
    q=zeros(1,robot.DOF)
end

h=figure(configuration.figure.robot);

%get adjusted view
[az,el] = view;

%retrieve the current zoom
zz=get(gca,'CameraViewAngle');

%delete figure
clf(h), grid on, hold on


xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')

%apply the stored view
view(az,el);
%apply the zoom
set(gca,'CameraViewAngle', zz);

origin = [0 0 0];
destination = [];
color = ['r' 'g' 'b' 'c' 'm' 'y' 'k'];

%in case of a parallel robot, call drawrobot3d for each serial link
if isfield(robot, 'parallel')
    drawrobot3d_par(robot, q);
    return;
end

if isfield(robot, 'skip_graphics')   
    return;
end


V=[];

T = robot.T0;
T2 = robot2.T0;
for i=1:robot.DOF+1,
    
    if robot.graphical.has_graphics
        draw_link(robot, i, T);
        draw_link(robot2, i, T2);
       % draw_link(robot1, i, T2); 
    else
        draw_link(robot, i, T);
        draw_link(robot2, i, T2);
        
        destination=T(1:3,4);
        col = randperm(6);
        line([origin(1) destination(1)],[origin(2) destination(2)],[origin(3) destination(3)], 'Color',  color(col(1)) ,'LineWidth', 3 );
        origin = destination;
        
        destination=T2(1:3,4);
        col = randperm(6);
        line([origin(1) destination(1)],[origin(2) destination(2)],[origin(3) destination(3)], 'Color',  color(col(1)) ,'LineWidth', 3 );
        origin = destination;
    end
    %draw D-H axes
    if robot.graphical.draw_axes
        draw_axes(T, sprintf('X_%d',i-1), sprintf('Y_%d',i-1), sprintf('Z_%d',i-1), robot.graphical.axes_scale);
        draw_axes(T2, sprintf('X_%d',i-1), sprintf('Y_%d',i-1), sprintf('Z_%d',i-1), robot2.graphical.axes_scale);
    end
    %obtain transformation to draw the next link
    if i<= robot.DOF
       T=T*dh(robot, q, i);
       T2=T2*dh(robot2, q2, i);
    end
end
%save for later use
T06robot=T;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DRAW A TOOL AT THE ROBOT'S END
% if there is a tool attached, draw it!
% repeat the prior commands
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(robot, 'tool')

    % Activate tool. Tools must be defined in a binary way.
    % This means that they are open or closed depending
    % on an joint
    if robot.tool.tool_open
        q_tool=ones(1,robot.tool.DOF);
    else
        q_tool=zeros(1,robot.tool.DOF);
    end

if robot2.tool.tool_open
        q2_tool=ones(1,robot2.tool.DOF);
    else
        q2_tool=zeros(1,robot2.tool.DOF);
end

    robot.tool.T0=T;
    robot2.tool.T0=T2;
    for i=1:robot.tool.DOF+1,
        if robot.tool.graphical.has_graphics
            draw_link(robot.tool, i, T);
        else
            draw_link(robot.tool, i, T);
            
            destination=T(1:3,4);
            col = randperm(6);
            line([origin(1) destination(1)],[origin(2) destination(2)],[origin(3) destination(3)], 'Color',  color(col(1)) ,'LineWidth', 3 );
            origin = destination;
        end
    end


for i=1:robot2.tool.DOF+1,
        if robot2.tool.graphical.has_graphics
            draw_link(robot2.tool, i, T2);
        else
            draw_link(robot2.tool, i, T2);
            
            destination=T2(1:3,4);
            col = randperm(6);
            line([origin(1) destination(1)],[origin(2) destination(2)],[origin(3) destination(3)], 'Color',  color(col(1)) ,'LineWidth', 3 );
            origin = destination;
        end
%      if robot.tool.graphical.draw_axes
        %   draw_axes(T, sprintf('X_{tool%d}',i-1), sprintf('Y_{tool%d}',i-1), sprintf('Z_{tool%d}',i-1));
%         end
        %obtain transformation to draw the next link
        if (i)<= robot.tool.DOF
            T=T*dh(robot.tool, q_tool, i);
        end

if (i)<= robot2.tool.DOF
            T2=T*dh(robot2.tool, q2_tool, i);
        end
    end
    %finally draw the Tool Center Point (TCP)
   % if (robot.tool.graphical.draw_axes) && (robot2.tool.graphical.draw_axes)
      %  draw_axes(T06robot*robot.tool.TCP, 'X_{toolTCP}', 'Y_{toolTCP}', 'Z_{toolTCP}');
      %  draw_axes(T06robot*robot2.tool.TCP, 'X_{toolTCP}', 'Y_{toolTCP}', 'Z_{toolTCP}');  % solo ejes
    %end
    


    %save for later use
    T07=T06robot*robot.tool.TCP;
    %T06=T05robot*robot2.tool.TCP;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  draw equipment such as tables, conveyor velts... etc
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(robot, 'equipment')
    for i=1:length(robot.equipment),
        if robot.equipment{i}.graphical.has_graphics
            draw_link(robot.equipment{i}, 1, robot.equipment{i}.T0);
        end
        if robot.equipment{i}.graphical.draw_axes
            draw_axes(robot.equipment{i}.T0, sprintf('X_{env%d}',0), sprintf('Y_{env%d}',0), sprintf('Z_{env%d}',0));
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DRAW A PIECE GRABBED
% this draws a piece grabbed at the robots end, or at the last known position
% the variable robot.tool_activated=1 asumes that the piece has been grabbed by the end tool
% robot.tool_activated==0 maintains the last known location for the pieced
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(robot, 'piece')
    for i=1:size(robot.piece,2),
        pieza_dibujar = robot.piece{i};
    if isfield(pieza_dibujar, 'graphical')

        if pieza_dibujar.graphical.has_graphics
            if isfield(robot, 'tool')
                if robot.tool.piece_gripped==1
                    %update the last known position and orientation of the robot
                    %Trel is computed when the simulation_grip_piece function is
                    %executed. Trel is the relative position and orientation of
                    %the tool and the piece that assures that the piece is
                    %picked at a constant and visually effective orientation.
                    draw_link(pieza_dibujar, 1, T07*(robot.tool.Trel));
                else
                    draw_link(pieza_dibujar, 1, pieza_dibujar.T0);
                end
            end
        else
            draw_link(pieza_dibujar, 1, pieza_dibujar.T0);
        end
        if pieza_dibujar.graphical.draw_axes
            draw_axes(pieza_dibujar.T0, sprintf('X_{piece%d}',0), sprintf('Y_{piece%d}',0), sprintf('Z_{piece%d}',0));
        end

    end
    end
end



