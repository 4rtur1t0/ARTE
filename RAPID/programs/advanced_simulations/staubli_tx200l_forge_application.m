% Matlab script for RAPID equivalent commands
% please note that variables should be
% declared as global as in RAPID syntaxis
% this script demonstrates the use of functions in the matlab script. This
% script can be translated to RAPID by means of the matlab2RAPID script.
%
% IN ORDER TO SIMULATE THE PROGRAM:
%   A) FIRST, LOAD A ROBOT
%       robot = load_robot('staubli','tx200l');
%       OR
%       robot = load_robot('staubli','tx200l');
%   B) NEXT, LOAD SOME EQUIPMENT.
%       robot.equipment = load_robot('equipment','Oven');
%      
%   C) NOW, LOAD AN END TOOL
%       robot.tool= load_robot('equipment','end_tools/vacuum_1');
%   D) FINALLY, LOAD A PIECE TO GRAB BY THE ROBOT
%       robot.piece=load_robot('equipment','engranaje');
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

function staubli_tx200l_forge_application
global robot TD_gripper RT_initial RT_approach1 RT_grip RT_approach2 RT_approach3 RT_release

%Comment the following lines to avoid loading the robot at every simulation
robot = load_robot('STAUBLI','tx200l');
robot.equipment{1} = load_robot('equipment','oven');
robot.tool= load_robot('equipment','end_tools/vacuum_1');
robot.piece{1}=load_robot('equipment','gear');


%init the position of the piece at the beginning of the simulation
robot.piece{1}.T0(1:3,4)=[2 1 0.2]';
%robot.tool.piece_gripped=0;
drawrobot3d(robot, robot.q);


%define the tool
%In RAPID this is done by means of the tooldata structure
TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

%define target points FOR SIMULATION
RT_approach1=[[1.7012, -0.9079, 0.9405],[0.0239, 0.2986, 0.9540, -0.0114], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_initial=[[1.7012, -0.9079, 0.9405],[0.0239, 0.2986, 0.9540, -0.0114], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_grip=[[1.7012, -0.9079, 0.5405],[0.0239, 0.2986, 0.9540, -0.0114], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_approach2=[[1.7543, 0.4186, 0.7337],[0.0140, 0.8807, 0.4728, 0.0272], [0, -1, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_approach3=[[0.8312, 2.0838, 0.7332],[0.0191, 0.7430, 0.6689, -0.0129], [0, -1, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_release=[[0.8312, 2.0838, 0.5332],[0.0191, 0.7430, 0.6689, -0.0129], [0, -1, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];%CONST robtarget RT_grip:=[[-97.01,-493.4,246.8],[1.5E-05,-0.707114,-0.7071,-2E-06],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
%CONST robtarget RT_approach2:=[[532.11,-422.58,368.01],[6.1E-05,-0.707117,-0.707097,1.5E-05],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
%CONST robtarget RT_release:=[[503.36,-442.47,356.87],[0.000279,0.707036,0.707178,0.000235],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];


main
end

function main()

global TD_gripper RT_initial RT_approach1 RT_grip  RT_approach2 RT_approach3 RT_release
%close the tool
simulation_close_tool; %Set do1;

%move to the initial point
MoveJ(RT_initial, 'vmax' , 'fine' , TD_gripper, 'wobj0');

% Now open the tool
simulation_open_tool; %Reset do1; 

%Move to the approaching point
MoveL(RT_approach1, 'vmax' , 'fine' , TD_gripper, 'wobj0');




%Now, go down to the grabbing target point and
MoveL(RT_grip, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%and close the tool and grip the piece. These two functions
% must be called to simulate that the gripper has the piece grabbed and the
%tool is closed
simulation_close_tool; %Set do1;
simulation_grip_piece;

%Now go to the same approach point so that collisions with the table are
%avoided
MoveL(RT_approach1, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%Move to the approach next to the packaging area
MoveJ(RT_approach2, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveJ(RT_approach3, 'vmax' , 'fine' , TD_gripper, 'wobj0');


%go down to the release point inside the box
MoveL(RT_release, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%yes, release the piece
simulation_open_tool; %Reset do1; 
simulation_release_piece; %Reset do1; 

%now, go up
MoveL(RT_approach3, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%Now, go back to the initial point
MoveJ(RT_initial, 'vmax' , 'fine' , TD_gripper, 'wobj0');

end