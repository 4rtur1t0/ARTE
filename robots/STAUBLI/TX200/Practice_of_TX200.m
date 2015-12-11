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
%       robot.equipment = load_robot('equipment','tables/table_two_areas');
%      
%   C) NOW, LOAD AN END TOOL
%       robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
%   D) FINALLY, LOAD A PIECE TO GRAB BY THE ROBOT
%       robot.piece=load_robot('equipment','cylinders/cylinder_tiny');
%
%   E) IF NECESSARY, CHANGE THE POSITION AND ORIENTATION OF THE piece,
%   relative to the robot's base reference system.
%
%       robot.piece.T0= [1 0 0 -0.1;
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

function Practice_of_TX200

global robot TD_gripper RT_init RT_aprox1 RT_cog1 RT_aprox2 RT_cog2 RT_inter

%Comment the following lines to avoid loading the robot at every simulation
robot = load_robot('STAUBLI','TX200');
robot.tool= load_robot('equipment','end_tools/vacuum_1');
robot.equipment=load_robot('equipment','pallet');
robot.piece = load_robot('equipment','box_big');

%init the position of the piece at the beginning of the simulation
robot.equipment.T0(1:3,4)=[2.5 0 0]';
robot.piece.T0(1:3,4)=[-3 -.5 0]';
%robot.tool.piece_gripped=0;
drawrobot3d(robot, robot.q);




RT_init=[[-0.5430, -0.0000, 1.7860],[1.0000, 0.0000, -0.0000, -0.0000], [0, 0, 0, 6], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_aprox1=[[1.8483, -0.0000, 1.5966],[0.6946, -0.0000, 0.7194, -0.0000], [0, -1, 0, 2], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_cog1=[[1.7956, -0.0000, 1.6283],[0.7062, -0.0000, 0.7080, -0.0000], [0, -1, 0, 2], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_aprox2=[[-1.7070, 0.0000, 1.7383],[0.0000, -0.6763, -0.0000, 0.7366], [2, 0, -1, 2], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_cog2=[[-1.7142, 0.0000, 1.7118],[0.0000, -0.7066, -0.0000, 0.7076], [2, 0, -1, 2], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_inter=[[-0.0280, 1.7140, 1.7118],[0.4963, -0.5037, 0.4955, 0.5044], [1, -1, -1, 2], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];




%define the tool
%In RAPID this is done by means of the tooldata structure
TD_gripper=[1,[[0,0,0.425],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

%define target points FOR SIMULATION


%replace the above points TO PROGRAM THE REAL ROBOT.
%CONST robtarget RT_initial:=[[500,-400,500],[0.427269,0.289111,0.84406,-0.14635],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
%CONST robtarget RT_approach1:=[[-100,-500,400],[0,0.70711,0.70711,0],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
%CONST robtarget RT_grip:=[[-97.01,-493.4,246.8],[1.5E-05,-0.707114,-0.7071,-2E-06],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
%CONST robtarget RT_approach2:=[[532.11,-422.58,368.01],[6.1E-05,-0.707117,-0.707097,1.5E-05],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
%CONST robtarget RT_release:=[[503.36,-442.47,356.87],[0.000279,0.707036,0.707178,0.000235],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];


main
end

function main()

global robot TD_gripper RT_init RT_aprox1 RT_cog1 RT_aprox2 RT_cog2 RT_inter
%close the tool
simulation_close_tool; %Set do1;

%move to the initial point
MoveJ(RT_init, 'vmax' , 'fine' , TD_gripper, 'wobj0');

% Now open the tool
simulation_open_tool; %Reset do1; 

%Move to the approaching point
MoveJ(RT_aprox1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
%Now, go down to the grabbing target point and
MoveL(RT_cog1, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%and close the tool and grip the piece. These two functions
% must be called to simulate that the gripper has the piece grabbed and the
%tool is closed
simulation_close_tool; %Set do1;
simulation_grip_piece;

%Now go to the same approach point so that collisions with the table are
%avoided
MoveL(RT_aprox2, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%Move to the approach next to the packaging area
MoveC(RT_inter, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveC(RT_aprox2, 'vmax' , 'fine' , TD_gripper, 'wobj0');
%go down to the release point inside the box
MoveL(RT_cog2, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%yes, release the piece
simulation_open_tool; %Reset do1; 
simulation_release_piece; %Reset do1; 

%now, go up
MoveL(RT_aprox2, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%Now, go back to the initial point
MoveJ(RT_init, 'vmax' , 'fine' , TD_gripper, 'wobj0');

end