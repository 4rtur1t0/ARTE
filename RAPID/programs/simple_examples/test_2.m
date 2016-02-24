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

function test_2
global TD_tool0 RT_tp1 RT_tp2 RT_tp3 JV_q0

TD_tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

%initial position
JV_q0=[0 0 0 0 0 0]';
%target points
RT_tp1=[[0.3,0.0089,1.0195],[0.2175,0.0971,-0.0689,0.9688],[1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
RT_tp2=[[0.7150,-0.2000,0.5120],[0.7071,0.0,0.7071,0.0000],[-1,-1,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
RT_tp3=[[0.8,0.000,0.3],[0.7071,0.0,0.7071,0.0000],[-1,-1,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

main
end

function main()

global TD_tool0 RT_tp1 RT_tp2 RT_tp3 JV_q0

%The MoveAbsJ function performs a joint coordinate planning to the
%specified joint values
 MoveAbsJ(JV_q0, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveJ(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveJ(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 %Return to tp1
 MoveJ(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 
 MoveC(RT_tp2, RT_tp3, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 
 %use the Offs function to move relative to tp1
 % The Offs function makes a relative displacement in X, Y and Z directions
 %Please note that, by using Offs, the specified axes configuration may
 %differ from the one specified in tp1.
 MoveJ(Offs(RT_tp1,0.1,0,-0.1), 'vmax' , 'fine' , TD_tool0, 'wobj0');
 
 %use MoveL inside a Loop, moving incrementally
 for i=1:4
     MoveL(Offs(RT_tp1,0.1,i*0.1,-0.2), 'vmax' , 'fine' , TD_tool0, 'wobj0');
 end
 
end