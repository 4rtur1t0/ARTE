% Matlab script for RAPID equivalent commands
% please note that variables should be
% declared as global as in RAPID syntaxis
% this script demonstrates the use of functions in the matlab script. This
% script can be translated to RAPID by means of the matlab2RAPID script.
%
% IN ORDER TO SIMULATE THE PROGRAM:
%   A) FIRST, LOAD A ROBOT
%       robot = load_robot('YASKAWA','yaskawa_motoman_es165d_100');
%       
%   B) NEXT, LOAD SOME EQUIPMENT.
%       robot.equipment{1} = load_robot('equipment','conveyor_belt1');
%      
%   C) NOW, LOAD AN END TOOL
%       robot.tool= load_robot('equipment','end_tools/vacuum_1');
%   D) FINALLY, LOAD A PIECE TO GRAB BY THE ROBOT
%       robot.piece=load_robot('equipment','aluminum_plate1');
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

function robot_test
global robot TD_vacuum RT_initial RT_approach1 RT_vacuum_catch RT_approach2 RT_vacuum_release RT_transition1 RT_transition2 RT_transition3

%Comment the following lines to avoid loading the robot at every simulation
robot = load_robot('MOTOMAN','ES165D_100');
robot.equipment{1} = load_robot('equipment','conveyor_belt1');
robot.tool= load_robot('equipment','end_tools/vacuum_1');
robot.piece{1}=load_robot('equipment','aluminum_plate1');


adjust_view(robot);
fprintf('Press any key to continue')
%robot.tool.piece_gripped=0;
drawrobot3d(robot, robot.q);

%define the tool
%In RAPID this is done by means of the tooldata structure
TD_vacuum=[1,[[0,0,0.4],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

%define target points FOR SIMULATION
RT_initial=[[2.5250, -0.0000, 2.0500],[0.7071, -0.0000, 0.7071, -0.0000], [0, -1, 2, 2], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_approach1=[[2.2000, 0.0000, 1.0000],[0.0000, 0.7071, 0.7071, -0.0000], [0, -1, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_vacuum_catch=[[2.2000, 0.0000, 0.8510],[0.0000, 0.7071, 0.7071, -0.0000], [0, -1, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_transition1=[[2.2000, 0.0000, 1.3000],[0.0000, -0.0000, 1.0000, -0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_transition2=[[1.1000, -1.4600, 1.3000],[0.0000, 0.0000, 1.0000, 0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_transition3=[[0.7000, -1.4000, 1.5000],[0.0000, -0.0000, 1.0000, -0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_approach2=[[0.0000, -1.5000, 0.4000],[0.0000, 0.7071, 0.7071, 0.0000], [-2, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_vacuum_release=[[0.0000, -1.5000, 0.0510],[0.0000, 0.7071, 0.7071, 0.0000], [-2, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

main;
end

function main()

global TD_vacuum RT_initial RT_approach1 RT_vacuum_catch RT_approach2 RT_vacuum_release RT_transition1 RT_transition2 RT_transition3

%close the tool
simulation_close_tool; %Set do1;

%move to the initial point
MoveJ(RT_initial, 'vmax' , 'fine' ,TD_vacuum, 'wobj0');

% Now open the tool
simulation_open_tool; %Reset do1; 

%Move to the approaching point
MoveJ(RT_approach1, 'vmax' , 'fine' ,TD_vacuum, 'wobj0');
%Now, go down to the grabbing target point and
MoveL(RT_vacuum_catch, 'vmax' , 'fine' ,TD_vacuum, 'wobj0');

%and close the tool and grip the piece. These two functions
% must be called to simulate that the gripper has the piece grabbed and the
%tool is closed
simulation_close_tool; %Set do1;
simulation_grip_piece;

%Move to a transition point to secure the integrity of the conveyor belt
MoveL(RT_transition1,'vmax','fine',TD_vacuum,'wobj0');

MoveL(RT_transition2,'vmax','fine',TD_vacuum,'wobj0');

%Move to the approach next to the packaging area
MoveJ(RT_approach2, 'vmax' , 'fine' ,TD_vacuum, 'wobj0');
%go down to the release point inside the box
MoveL(RT_vacuum_release, 'vmax' , 'fine' ,TD_vacuum, 'wobj0');

%yes, release the piece
simulation_open_tool; %Reset do1; 
simulation_release_piece; %Reset do1; 

%now, go up
MoveL(RT_approach2, 'vmax' , 'fine' ,TD_vacuum, 'wobj0');

%Move to a transition point to secure the integrity of the conveyor belt
MoveJ(RT_transition3,'vmax','fine',TD_vacuum,'wobj0');

%Now, go back to the initial point
MoveJ(RT_initial, 'vmax' , 'fine' ,TD_vacuum, 'wobj0');

end