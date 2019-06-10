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
%   Juan Carlos Ruzafa Moraga
%   Luis Palafox Catral
%   Adrian Verdu Correcher
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function viper_s650_painting_bowl
global robot tool0 RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8

%Cargamos ruta libreria y la inicializamos, modificar ruta para otro
%dispositivo.
cd('C:/Users/Juanky/Documents/MATLAB/arte');
init_lib;
%Cargamos el robot y los elementos necesarios
robot = load_robot('ADEPT','VIPER_s650');
robot.tool= load_robot('equipment','end_tools/paint_gun');
robot.piece{1}=load_robot('equipment','miscelanea/cat_bowl');
robot.equipment{1} = load_robot('equipment','tables/table_extended');
%Colocamos en sus posiciones iniciales la cinta transportadora y el objeto
%a pintar

robot.equipment{1}.T0(1:3,4)=[-1 -0.7 0]';
robot.piece{1}.T0(1:3,4)=[0 -0.4 0.2]';

%Dibujamos
 
drawrobot3d(robot, robot.q);

tool0=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

RT_tp1=[[0.5500, -0.0000, 0.6950],[0.7071, -0.0000, 0.7071, -0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[0.3233, -0.4450, 0.6950],[0.6300, 0.3210, 0.6300, -0.3210], [-1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[0.1376, -0.4800, 0.5064],[0.0406, 0.6010, 0.7976, -0.0306], [-1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[-0.2025, -0.3574, 0.4313],[0.1080, 0.8783, 0.3806, 0.2683], [-2, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[0.0072, -0.2618, 0.3413],[0.2046, 0.5832, 0.7296, -0.2926], [-1, -2, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp6=[[0.1141, -0.3470, 0.3964],[0.1315, 0.0845, 0.8953, -0.4172], [-1, -2, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp7=[[-0.0141, -0.2420, 0.3332],[0.2467, 0.6402, 0.6904, -0.2293], [-2, -2, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp8=[[0.1516, -0.5287, 0.6950],[0.5647, 0.4255, 0.5647, -0.4255], [-1, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

%SYSTEM PAUSE
adjust_view(robot)

simulation_open_tool;

MoveL(RT_tp1, 'vmax' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp2, 'vmax' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp3, 'v1000' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp4, 'vmax' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp5, 'vmax' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp6, 'v1000' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp7, 'vmax' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp8, 'vmax' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp1, 'vmax' , 'fine' , tool0, 'wobj0');

