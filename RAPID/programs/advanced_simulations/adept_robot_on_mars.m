% Matlab script for RAPID equivalent commands
% please note that variables should be
% declared as global as in RAPID syntaxis
% this script demonstrates the use of functions in the matlab script. This
% script can be translated to RAPID by means of the matlab2RAPID script.
%
% IN ORDER TO SIMULATE THE PROGRAM:
%   A) FIRST, LOAD A ROBOT
%       robot = load_robot('abb','irb140');
%       OR
%       robot = load_robot('abb','irb52');
%   B) NEXT, LOAD SOME EQUIPMENT.
%       robot.equipment{1} = load_robot('equipment','tables/table_two_areas');
%      
%   C) NOW, LOAD AN END TOOL
%       robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
%   D) FINALLY, LOAD A PIECE TO GRAB BY THE ROBOT
%       robot.piece{1}=load_robot('equipment','cylinders/cylinder_tiny');
%
%   E) IF NECESSARY, CHANGE THE POSITION AND ORIENTATION OF THE piece,
%   relative to the robot's base reference system.
%
%       robot.piece{1}.T0= [1 0 0 -0.1;
%                        0 1 0 -0.5;
%                        0 0 1 0.2;
%                        0 0 0 1]; 
%
% during the simulation, call simulation_open_tool; to open the tool and 
% simulation_close_tool; to close it.
%
% To grip the piece, call simulation_grip_piece; and
% simulation_release_piece to release it.
%
% The call to each function must be in a correct, otherwise, the result may be
% undesired, thus, typically the correct sequence is:
% simulation_open_tool;
% (approach the piece to grab with MoveL or MoveJ)
% simulation_close_tool;
% simulation_grip_piece; --> the piece will be drawn with the robot
%
% move to a different place with MoveJ or MoveL
% simulation_open_tool;
% simulation_release_piece
%
% In RAPID, consider the use of ConfJ on, or ConfJ off, or ConfL on, or ConfL off
% in the case the controller avoids the change of configurations between target points


% Ruben Marco Navarro
% Jose Daniel Sanchez Albert


function adept_robot_on_mars
global robot TD_gripper  RT_abrir_griper RT_cerrar_griper RT_transporte_1 

%Comment the following lines to avoid loading the robot at every simulation
%robot = load_robot('abb','irb140');
%robot.equipment{1} = load_robot('equipment','tables/table_two_areas');
%robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
%robot.piece=load_robot('equipment','cylinders/cylinder_tiny');

robot = load_robot('EPSON','C8L');
robot.equipment{1} = load_robot('equipment/objects','marte');
robot.equipment{2} = load_robot('equipment/objects','cohete');
robot.equipment{3} = load_robot('equipment/objects','rover');
robot.equipment{4} = load_robot('equipment/objects','roca_grande');
robot.tool= load_robot('equipment','end_tools/parallel_gripper_1');
robot.equipment{1}.T0(1:3,4)=[0 4 -0.6]';
robot.equipment{2}.T0(1:3,4)=[6 0 -0.15]';
robot.equipment{3}.T0(1:3,4)=[-0.2 0 -0.15]';
robot.equipment{4}.T0(1:3,4)=[0.6 0.5 -0.15]';
robot.T0(1:3,4)=[0 0 0]';
robot.piece{1}= load_robot('equipment/objects','roca');
robot.piece{1}.T0(1:3,4)=[0.6 0.4 0.25]';


%init the position of the piece at the beginning of the simulation
%robot.piece.T0(1:3,4)=[-0.1 -0.5 0.2]';
%robot.tool.piece_gripped=0;
drawrobot3d(robot, robot.q);
adjust_view(robot)
drawrobot3d(robot, robot.q);
%define the tool
%In RAPID this is done by means of the tooldata structure
TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

%define target points FOR SIMULATION



RT_abrir_griper=[[0.5700, 0.3900, 0.4820],[0.0000, -0.7071, -0.0000, -0.7071], [0, 0, -1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_cerrar_griper=[[0.5700, 0.3900, 0.3420],[0.0000, -0.7071, -0.0000, -0.7071], [0, 0, -1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_transporte_1=[[0.4200, -0.0000, 0.9020],[0.0000, 0.7071, 0.0000, 0.7071], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];



main
end

function main()

global robot TD_gripper  RT_abrir_griper RT_cerrar_griper RT_transporte_1 
%close the tool
simulation_close_tool; %Set do1;

%move to the initial point
%MoveJ(RT_initial, 'vmax' , 'fine' , TD_gripper, 'wobj0');

% Now open the tool
%simulation_open_tool; %Reset do1; 

%Move to the approaching point
%MoveJ(RT_approach1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
%Now, go down to the grabbing target point and
%MoveL(RT_grip, 'vmax' , 'fine' , TD_gripper, 'wobj0');


MoveJ(RT_abrir_griper, 'vmax' , 'fine' , TD_gripper,'wobj0');
simulation_open_tool;
MoveJ(RT_cerrar_griper, 'vmax' , 'fine' , TD_gripper,'wobj0');
simulation_close_tool;
simulation_grip_piece;
MoveJ(RT_transporte_1, 'vmax' , 'fine' , TD_gripper,'wobj0');

%Movimiento lineal robot y coche
for i = 0:0.1:5.5
    robot.T0(1:3,4)=[i 0 0]'
    robot.equipment{3}.T0(1:3,4)=[(i-0.2) 0 -0.15]'
    drawrobot3d(robot, robot.q);
end
simulation_open_tool;
simulation_release_piece;

%Retroceso robot y coche
for i = 5.5:-0.1:4.8
    robot.T0(1:3,4)=[i 0 0]'
    robot.equipment{3}.T0(1:3,4)=[(i-0.2) 0 -0.15]'
    drawrobot3d(robot, robot.q);
end

%Despegue cohete
for i = 0:0.3:6
    robot.equipment{2}.T0(1:3,4)=[6 0 (-0.15+i)]'
    robot.piece{1}.T0(1:3,4)=[6 0 +i]
    drawrobot3d(robot, robot.q);
end
%MoveL(RT_posicion2_robot, 'vmax' , 'fine' , robot, 'wobj0');

%and close the tool and grip the piece. These two functions
% must be called to simulate that the gripper has the piece grabbed and the
%tool is closed
%simulation_close_tool; %Set do1;
%simulation_grip_piece;

%Now go to the same approach point so that collisions with the table are
%avoided
%MoveL(RT_approach1, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%Move to the approach next to the packaging area
%MoveJ(RT_approach2, 'vmax' , 'fine' , TD_gripper, 'wobj0');
%go down to the release point inside the box
%MoveL(RT_release, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%yes, release the piece
%simulation_open_tool; %Reset do1; 
%simulation_release_piece; %Reset do1; 

%now, go up
%MoveL(RT_approach2, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%Now, go back to the initial point
%MoveJ(RT_initial, 'vmax' , 'fine' , TD_gripper, 'wobj0');

end