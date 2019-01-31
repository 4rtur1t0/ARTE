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
%   Miriam Plaza Carrasco
%   Alejandro Cazorla Sánchez
%   Vicente Tormo Moragues
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function adept_viper_900_feeds_cat
global robot TD_gripper Pinic Pprec1 PCcaja Pcaja PPcaja Pn1 Pi1 Pi2 PCcuenco Pcuenco PSgato PCgato Pgato1 Pgato2 Pgato3 Pgato4

%Comment the following lines to avoid loading the robot at every simulation
robot = load_robot('ADEPT','Viper_900');
robot.equipment{1} = load_robot('equipment','tables/table_two_areas');
robot.equipment{2} = load_robot('equipment','miscelanea/cat_bowl');
robot.equipment{2}.T0(1:3,4)=[0.5 0.25 0]';
robot.equipment{3} = load_robot('equipment','miscelanea/pink_cat');
robot.equipment{3}.T0(1:3,4)=[1.3 0 0]';
robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
robot.piece{1}=load_robot('equipment','miscelanea/cereal_box');

%init the position of the piece at the beginning of the simulation
robot.piece{1}.T0(1:3,4)=[0.2 -0.3 0.2]';
%%robot.equipment.cat.T0(1:3,4)=[1.3 0 0]';
robot.piece{1}.piece_gripped=0;
drawrobot3d(robot, robot.q);
adjust_view(robot)
%mover gato %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drawrobot3d(robot, robot.q);
%define the tool
%In RAPID this is done by means of the tooldata structure
TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

%define target points FOR SIMULATION
Pinic=[[0.6880, -0.0000, 0.8100],[0.7071, -0.0000, 0.7071, -0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pprec1=[[0.0880, -0.3000, 0.5100],[0.7071, -0.0000, 0.7071, 0.0000], [-1, -1, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
PCcaja=[[0.0891, -0.2969, 0.2947],[0.7159, 0.0255, 0.6972, 0.0255], [-1, -1, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pcaja=[[0.2159, -0.2980, 0.2980],[0.7147, -0.0215, 0.6978, 0.0418], [-1, -1, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
PPcaja=[[0.2159, -0.2980, 0.3980],[0.7147, -0.0215, 0.6978, 0.0418], [-1, -1, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pn1=[[0.2319, -0.2830, 0.3999],[0.6885, 0.0136, 0.7248, -0.0221], [-1, -1, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pi1=[[0.4819, -0.0830, 0.3999],[0.6885, 0.0136, 0.7248, -0.0221], [-1, -1, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pi2=[[0.5319, 0.2170, 0.3999],[0.6885, 0.0136, 0.7248, -0.0221], [0, 0, -1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
PCcuenco=[[0.4732, 0.2653, 0.2871],[0.6701, -0.0386, 0.7410, 0.0199], [0, 0, -1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pcuenco=[[0.3761, 0.2804, 0.1386],[0.2670, -0.0933, 0.9575, 0.0560], [0, 0, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
PSgato=[[0.5417, 0.3480, 0.6089],[0.0030, 0.0030, 0.7091, 0.7051], [0, 1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
PCgato=[[0.5917, 0.3480, 0.4589],[0.6401, -0.6462, -0.2920, -0.2958], [0, 1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pgato1=[[0.5917, 0.3480, 0.4089],[0.6401, -0.6462, -0.2920, -0.2958], [0, 1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pgato2=[[0.6284, 0.3449, 0.3425],[0.6429, -0.6557, -0.2795, -0.2803], [0, 1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pgato3=[[0.6784, 0.3449, 0.2925],[0.6429, -0.6557, -0.2795, -0.2803], [0, 1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
Pgato4=[[0.6784, 0.3449, 0.3425],[0.6429, -0.6557, -0.2795, -0.2803], [0, 1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

main
end

function main()

global robot TD_gripper Pinic Pprec1 PCcaja Pcaja PPcaja Pn1 Pi1 Pi2 PCcuenco Pcuenco PSgato PCgato Pgato1 Pgato2 Pgato3 Pgato4
%open the tool
simulation_open_tool; %Set do1;

%move to the initial point
MoveJ(Pprec1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(PCcaja, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(Pcaja, 'v1000' , 'fine' , TD_gripper, 'wobj0');

% Now open the tool
simulation_close_tool; %Reset do1; 
simulation_grip_piece;

MoveL(PPcaja, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(Pn1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveC(Pi1, Pi2, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%semueve a cuenco
MoveJ(PCcuenco, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(Pcuenco, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(PCcuenco, 'vmax' , 'fine' , TD_gripper, 'wobj0');

%camino inverso
MoveC(Pi2, Pi1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(Pn1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(PPcaja, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(Pcaja, 'vmax' , 'fine' , TD_gripper, 'wobj0');
%abrir gripper
simulation_open_tool; %Set do1;
simulation_release_piece;

MoveL(PCcaja, 'vmax' , 'fine' , TD_gripper , 'wobj0');

MoveL(Pprec1, 'vmax' , 'fine' , TD_gripper , 'wobj0');

%cerrar gripper
simulation_close_tool; %Set do1;
%%%EL GATO SE MUEVE!!! TAMOS LOCOS
robot.equipment{3}.T0(1:3,4)=[1.15 0.05 0]';
drawrobot3d(robot, robot.q);
robot.equipment{3}.T0(1:3,4)=[1.05 0.1 0]';
drawrobot3d(robot, robot.q);
robot.equipment{3}.T0(1:3,4)=[1 0.12 0]';
drawrobot3d(robot, robot.q);
robot.equipment{3}.T0(1:3,4)=[0.9 0.15 0]';
drawrobot3d(robot, robot.q);
robot.equipment{3}.T0(1:3,4)=[0.75 0.2 0]';
drawrobot3d(robot, robot.q);
robot.equipment{3}.T0(1:3,4)=[0.7 0.25 0]';
drawrobot3d(robot, robot.q);


MoveJ(PSgato, 'vmax' , 'fine' , TD_gripper , 'wobj0');
MoveL(PCgato, 'vmax' , 'fine' , TD_gripper , 'wobj0');

for i=1:3
MoveL(Pgato1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveC(Pgato2, Pgato3 , 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(Pgato4, 'vmax' , 'fine' , TD_gripper , 'wobj0');
MoveL(Pgato1, 'vmax' , 'fine' , TD_gripper , 'wobj0');
end

MoveJ(Pinic, 'vmax' , 'fine', TD_gripper, 'wobj0');

robot.equipment{3}.T0=[1 0 0 0.2;
                      0 -1 0 0;
                      0 0 1 0.4
                      0 0 0 1];
drawrobot3d(robot, robot.q);

end