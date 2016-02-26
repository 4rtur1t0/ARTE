%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Adept Viper s1300 DEMO
%
%   Author: Santiago Ferre Satoca, Javier Ferre Satoca and
%   Cristian Gumpert Gomar - Universidad Miguel Hernández de Elche. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

function test_viper_s1300
    global robot
    global qua
    global milling_tool

    % Para que la ejecucion no sea tan lenta
    configuration.delta_time=0.04;

    q=[0 0 0 0 pi/2 0];

    % Configuración inicial
    robot=load_robot('ADEPT', 'Viper_s1300');
    punto=directkinematic(robot, [0 0 0 0 pi/2 0]);
    qua=T2quaternion(punto);

    robot.tool=load_robot('equipment/end_tools', 'milling_machine');
    robot.equipment=load_robot('equipment', 'aluminum_plate');
    drawrobot3d(robot, q);
    robot.graphical.draw_axes = 0;
    robot.tool.graphical.draw_axes = 0;
    
    adjust_view(robot)

    milling_tool=[1,[[robot.tool.TCP(1,4),robot.tool.TCP(2,4),robot.tool.TCP(3,4)],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]]; 

    main;
end

function main()
    circleA;
    circleB;  
    triangle;
    circleC;
    circleD;
    endpos;
end

function circleA()
    global qua milling_tool
    
    E=[[0.7 0.2033 0.2; 0.66 0.2433 0.2; 0.7 0.2833 0.2;0.74 0.2433 0.2]];

    RT_tp0=[[E(1,1:2) 0.4],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp1=[[E(1,:)],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp2=[[E(2,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp3=[[E(3,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp4=[[E(4,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];    
    
    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp1,RT_tp2, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveC(RT_tp2,RT_tp3, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveC(RT_tp3,RT_tp4, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveC(RT_tp4,RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveJ(RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');
end

function circleB()
    global qua milling_tool
    
    E=[[1.025 0.16 0.2; 0.985 0.2 0.2; 1.025 0.24 0.2;1.065 0.2 0.2]];

    RT_tp0=[[E(1,1:2) 0.4],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp1=[[E(1,:)],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp2=[[E(2,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp3=[[E(3,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp4=[[E(4,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp1,RT_tp2, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveC(RT_tp2,RT_tp3, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp3,RT_tp4, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp4,RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveJ(RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');
end

function circleC()
    global qua milling_tool
    
    E=[[0.7 -0.2033 0.2; 0.74 -0.2433 0.2; 0.7 -0.2833 0.2;0.66 -0.2433 0.2]];
    
    RT_tp1=[[E(1,:)],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp2=[[E(2,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp3=[[E(3,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp4=[[E(4,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp0=[[E(1,1:2) 0.4],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp1,RT_tp2, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveC(RT_tp2,RT_tp3, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp3,RT_tp4, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp4,RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveJ(RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');
end

function circleD()
    global qua milling_tool
    
    E=[[1.025 -0.16 0.2; 1.065 -0.2 0.2; 1.025 -0.24 0.2;0.985 -0.2 0.2]];

    RT_tp1=[[E(1,:)],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp2=[[E(2,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp3=[[E(3,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp4=[[E(4,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp0=[[E(1,1:2) 0.4],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');

    MoveC(RT_tp1,RT_tp2, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveC(RT_tp2,RT_tp3, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp3,RT_tp4, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveC(RT_tp4,RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveJ(RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp0, 'vmax' , 'z100' , milling_tool, 'wobj0');
end

function triangle()
    global qua milling_tool
    
    E=[[0.8 -0.0866 0.2; 0.8 0.0866 0.2; 0.95 0 0.2; 0.8 -0.0866 0.5]];

    RT_tp1=[[E(1,:)],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp2=[[E(2,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp3=[[E(3,:)],[qua], [-1, -1, -2, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    RT_tp4=[[E(4,:)],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    MoveL(RT_tp4, 'vmax' , 'z100' , milling_tool, 'wobj0');
    MoveL(RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp2, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp3, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp1, 'vmax' , 'fine' , milling_tool, 'wobj0');
    MoveL(RT_tp4, 'vmax' , 'z100' , milling_tool, 'wobj0');
end

function endpos()
    global qua milling_tool
   
    RT_tpend=[[0.7 0.2033 0.4],[qua], [0, -1, -1, 0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    MoveL(RT_tpend, 'vmax' , 'z100' , milling_tool, 'wobj0');
end