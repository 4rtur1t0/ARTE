%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Simulation courtesy of Vicente Quiles
%                          Andres Lopez 
%                          Pablo Valero  
%                           
%                   Universidad Miguel Hernández de Elche.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2015, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
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
function abb_irb6640_car_welding
global robot 
robot = load_robot('abb','irb6640');
robot.equipment{1} = load_robot('equipment','conveyor_belt');
robot.tool= load_robot('equipment','end_tools/spot_welding');
robot.piece=load_robot('equipment','bodywork2');


car_welding()

end


function car_welding()
global TD_tool0 RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 JV_q0

TD_tool0=[1,[[0,0,0.4],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

%initial position
JV_q0=[0 0 0 0 0 0]';
%target points
RT_tp1=[[1.9634, -1.1197, 1.1176],[0.2586, 0.4155, 0.7966, -0.3548], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[2.1670, -1.1049, 1.1050],[0.2532, 0.4036, 0.8039, -0.3561], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[2.3838, -1.0234, 1.0788],[0.2578, 0.4112, 0.7986, -0.3559], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[2.4090, -0.9687, 1.0489],[0.2578, 0.4112, 0.7986, -0.3559], [-1, -1, 0, 2], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[2.2000, 0.7499, 1.0500],[0.7046, 0.0122, 0.7093, -0.0137], [0, 0, -1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

main
end

function main()

global TD_tool0 RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 JV_q0 robot

for i=-7:0.5:2
robot.piece.T0 = [1 0 0 0;
             0 1 0 i;
             0 0 1 0;
             0 0 0 1];
    drawrobot3d(robot,robot.q);
end

MoveJ(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
simulation_close_tool
simulation_open_tool
MoveJ(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');
simulation_close_tool
simulation_open_tool
MoveJ(RT_tp3, 'vmax' , 'fine' , TD_tool0, 'wobj0');
simulation_close_tool
simulation_open_tool
MoveJ(RT_tp4, 'vmax' , 'fine' , TD_tool0, 'wobj0');
simulation_close_tool
simulation_open_tool

 MoveAbsJ(JV_q0, 'vmax' , 'fine' , TD_tool0, 'wobj0');

 for i=2:0.5:5
    robot.piece.T0 = [1 0 0 0;
             0 1 0 i;
             0 0 1 0;
             0 0 0 1];
drawrobot3d(robot,robot.q);
end

MoveJ(RT_tp5, 'vmax' , 'fine' , TD_tool0, 'wobj0');
simulation_close_tool
simulation_open_tool

MoveAbsJ(JV_q0, 'vmax' , 'fine' , TD_tool0, 'wobj0');

for i=5:0.5:13
robot.piece.T0 = [1 0 0 0;
             0 1 0 i;
             0 0 1 0;
             0 0 0 1];
drawrobot3d(robot,robot.q);
end

end