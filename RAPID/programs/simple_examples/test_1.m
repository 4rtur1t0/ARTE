% IN ORDER TO SIMULATE THE PROGRAM:
%   A) FIRST, LOAD A ROBOT
%       robot = load_robot('abb','irb140');
%   B) NEXT, LOAD SOME EQUIPMENT.
%       robot.equipment = load_robot('equipment','tables/table_small');
%       OR
%       robot.equipment = load_robot('equipment','bumper_cutting');
%   C) NOW, LOAD AN END TOOL
%       robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
%   D) FINALLY, LOAD A PIECE TO GRAB BY THE ROBOT
%       robot.piece=load_robot('equipment','cylinders/cylinder_tiny');
%
%   E) IF NECESSARY, CHANGE THE POSITION AND ORIENTATION OF THE ROBOT'S
%   BASE
%       robot.piece.T0= [1 0 0 -0.35;
%                        0 1 0 -0.55;
%                        0 0 1 0.2;
%                        0 0 0 1]; 
%
% during the simulation, call simulation_open_tool; to open the tool and 
% simulation_close_tool; to close it.
% To grip the piece, call simulation_grip_piece; and
% simulation_release_piece to release it.
% The call to each function must be correct, thus, typically the correct
% sequence is:
% simulation_open_tool;
% approach the piece to grab.
% simulation_close_tool;
% simulation_grip_piece; --> the piece will be drawn with the robot
% move to a different place
% simulation_open_tool;
% simulation_release_piece

function test_1

global RT_tp1 RT_tp2 TD_tool0

RT_tp1=[[0.4000, 0.3000, 0.7000],[1.0000, -0.0000, -0.0000, -0.0000], [0, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[0.1000, -0.4000, 0.7000],[0.0000, -0.0000, 0.0000, 1.0000], [-1, -1, -2, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

%definition of the end effector.
TD_tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

%finally, call the main function
main
end

function main
global RT_tp1 RT_tp2 TD_tool0

MoveJ(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveL(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveL(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');

end