%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab script for RAPID equivalent commands
% please note that variables should be
% declared as global as in RAPID syntaxis
% this script demonstrates the use of functions in the matlab script. This
% script can be translated to RAPID by means of the matlab2RAPID script.
%
% This script includes all the robot, equipment, pieces and end tool
% loading necessary. It can't be used with another robot without modifying
% this script.
% For the proper functioning of this script you shall use the modified
% function drawrobot3d so two extra equipment can be added.
% The authors of this scripts are:
%   Amalia Samper
%   Carlota Yañez
%   Yessica Millán
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function kuka_kr60_3_application
global robot TD_gripper  tp1 tp2 tp3 tp4 tp5 tp6 tp7 tp8 tp9 tp10 tp11 tp12 tp13 tp14 tp15 tp16 tp17 tp18 tp19 tp20 tp21 tp22 tp23 tp24 tp25 tp26 tp27 tp28 tp29 tp30 tp31

%Comment the following lines to avoid loading the robot at every simulation
robot = load_robot('KUKA','KR60_3');
robot.equipment{1} = load_robot('equipment','tables/table_1');
robot.tool= load_robot('equipment','end_tools/paint_gun');
robot.piece=load_robot('equipment','objects/piece');


%init the position of the piece at the beginning of the simulation
robot.piece.T0(1:3,4)=[1.7 0.05 0.8]';
%robot.tool.piece_gripped=0;
drawrobot3d(robot, robot.q);
adjust_view(robot)


drawrobot3d(robot, robot.q);
%define the tool
%In RAPID this is done by means of the tooldata structure
TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

%define target points FOR SIMULATION
tp1=[[1.7578, 0.0000, 1.2580],[0.0000, 0.8430, 0.0000, 0.5379], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp2=[[1.5287, 0.0000, 1.1023],[0.0000, 0.9996, 0.0000, 0.0292], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp3=[[1.5646, 0.0000, 1.1692],[0.0000, 0.9976, 0.0000, 0.0690], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp4=[[1.5976, 0.0000, 1.0254],[0.0000, 0.9999, 0.0000, 0.0107], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp5=[[1.6506, 0.0000, 1.1195],[0.0000, 0.9977, 0.0000, 0.0674], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp6=[[1.6752, 0.0000, 0.9827],[0.0000, 0.9999, 0.0000, 0.0154], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp7=[[1.6849, 0.0000, 0.8643],[0.0000, 0.9996, -0.0000, -0.0291], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp8=[[1.4455, -0.0000, 1.5794],[0.0000, 0.9671, 0.0000, 0.2543], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp9=[[1.2341, -0.7525, 1.5794],[0.0688, 0.9311, -0.2615, 0.2448], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp10=[[0.2226, -1.4282, 1.5794],[0.1654, 0.7346, -0.6290, 0.1931], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp11=[[0.2525, -1.6197, 1.1645],[0.0551, 0.7569, -0.6480, 0.0644], [0, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp12=[[0.0946, -1.6365, 1.1645],[0.0582, 0.7246, -0.6839, 0.0616], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp13=[[0.0958, -1.6570, 1.0775],[0.0353, 0.7263, -0.6855, 0.0374], [0, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp14=[[0.0931, -1.6098, 0.9981],[0.0020, 0.7272, -0.6864, 0.0021], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp15=[[0.0938, -1.6228, 0.7931],[0.0533, 0.7250, -0.6843, -0.0565], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp16=[[0.0875, -1.5123, 0.6910],[0.1072, 0.7183, -0.6780, -0.1136], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp17=[[0.0842, -1.4561, 0.4364],[0.1820, 0.7012, -0.6618, -0.1929], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp18=[[0.0864, -1.4935, 1.0578],[0.0004, 0.7272, -0.6864, 0.0004], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp19=[[-0.4400, -1.4298, 1.0578],[0.0004, -0.5941, 0.8044, -0.0003], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp20=[[-1.1744, -0.9266, 1.0578],[0.0005, -0.3278, 0.9447, -0.0002], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp21=[[-1.4680, -0.2877, 1.0578],[0.0005, -0.0966, 0.9953, -0.0001], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp22=[[-1.3115, -0.2570, 0.8750],[0.1262, -0.0958, 0.9873, 0.0123], [1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp23=[[-1.3678, -0.2680, 0.8666],[0.0195, -0.0966, 0.9951, 0.0019], [1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp24=[[-1.1727, -0.2298, 0.7073],[0.1493, -0.0955, 0.9841, 0.0145], [1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp25=[[-1.2386, -0.2427, 0.6955],[0.0239, -0.0966, 0.9950, 0.0023], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp26=[[-1.0484, -0.2055, 0.6022],[0.1265, -0.0958, 0.9873, 0.0123], [1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp27=[[-1.1027, -0.2161, 0.5938],[0.0235, -0.0966, 0.9950, 0.0023], [1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp28=[[-1.1247, -0.2204, 0.5936],[0.0179, -0.0966, 0.9952, -0.0017], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp29=[[-1.1470, -0.2248, 0.7039],[0.0856, -0.0962, 0.9916, -0.0083], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp30=[[-1.1531, -0.2260, 0.8612],[0.1797, -0.0950, 0.9790, -0.0174], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tp31=[[-1.3737, -0.2692, 1.1491],[0.3429, -0.0907, 0.9344, -0.0333], [1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
main
end

function main()

global TD_gripper  tp1 tp2 tp3 tp4 tp5 tp6 tp7 tp8 tp9 tp10 tp11 tp12 tp13 tp14 tp15 tp16 tp17 tp18 tp19 tp20 tp21 tp22 tp23 tp24 tp25 tp26 tp27 tp28 tp29 tp30 tp31


%move to the initial point
MoveL(tp1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp2, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp3, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp4, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp5, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp6, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp7, 'vmax' , 'fine' , TD_gripper, 'wobj0');

simulation_grip_piece;


MoveL(tp8, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp9, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp10, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp11, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp12, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp13, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp14, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp15, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp16, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp17, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp18, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp19, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp20, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp21, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp22, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp23, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp24, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp25, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp26, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp27, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp28, 'vmax' , 'fine' , TD_gripper, 'wobj0');

simulation_release_piece;

MoveL(tp29, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp30, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(tp31, 'vmax' , 'fine' , TD_gripper, 'wobj0');



end