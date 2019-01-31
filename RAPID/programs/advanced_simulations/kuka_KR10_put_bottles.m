
function kuka_KR10_put_bottles
global robot TD_gripper  RT_Inicio RT_P1 RT_rotpinza RT_P2 RT_Botella RT_P3 RT_P4 RT_P5 RT_Caja RT_Fin
%Comment the following lines to avoid loading the robot at every simulation
robot = load_robot('kuka','KR10_R1100_sixx');
robot.equipment{1} = load_robot('equipment','tables/table_extended');
robot.equipment{2} = load_robot('equipment/objects','fruit_box');
robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
robot.piece{1}=load_robot('equipment/objects','water_bottle');
 
%init the position of the piece at the beginning of the simulation
robot.piece{1}.T0(1:3,4)=[0.75 -0.55 0.2]';

%robot.tool.piece_gripped=0;
drawrobot3d(robot, robot.q);
adjust_view(robot)

 %define the tool
%In RAPID this is done by means of the tooldata structure
TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];
 
%define target points FOR SIMULATION
RT_Inicio=[[0.7450, -0.0000, 0.9950],[0.7071, -0.0000, 0.7071, -0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_P1=[[0.7750, -0.4000, 0.3200],[0.0000, -0.0000, 1.0000, 0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_rotpinza=[[0.7750, -0.4000, 0.3200],[0.0000, -0.7071, -0.7071, -0.0000], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_P2=[[0.7850, -0.4000, 0.3000],[0.0000, 0.7071, 0.7071, 0.0000], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_Botella=[[0.7850, -0.4000, 0.2400],[0.0000, 0.7071, 0.7071, -0.0000], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_P3=[[0.4750, -0.3700, 0.6900],[0.0000, 0.7071, 0.7071, 0.0000], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_P4=[[-0.0950, -0.4000, 0.6900],[0.0000, 0.7071, 0.0000, 0.7071], [-2, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_P5=[[-0.2450, -0.4500, 0.4500],[0.0000, 0.7071, 0.0000, 0.7071], [-2, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_Caja=[[-0.2450, -0.4500, 0.3500],[0.0000, 0.7071, 0.0000, 0.7071], [-2, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_Fin=[[0.7450, -0.0000, 0.9950],[0.7071, -0.0000, 0.7071, -0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

 
main
end
 
function main()
 
global robot TD_gripper  RT_Inicio RT_P1 RT_rotpinza RT_P2 RT_Botella RT_P3 RT_P4 RT_P5 RT_Caja RT_Fin
 
%move to the initial point
MoveJ(RT_P1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveJ(RT_rotpinza, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(RT_P2, 'v1000' , 'fine' , TD_gripper, 'wobj0');

%open the tool
simulation_open_tool; %Set do1;

MoveL(RT_Botella, 'v1000' , 'fine' , TD_gripper, 'wobj0');

% Now close the tool
simulation_close_tool; %Reset do1; 
simulation_grip_piece;

MoveL(RT_P2, 'v1000' , 'fine' , TD_gripper, 'wobj0');
 
MoveJ(RT_P3, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveJ(RT_P4, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveJ(RT_P5, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveL(RT_Caja, 'v1000' , 'fine' , TD_gripper, 'wobj0');

%open the tool
simulation_open_tool; %Set do1;
simulation_release_piece;

MoveL(RT_P5, 'v1000' , 'fine' , TD_gripper, 'wobj0');
%cerrar gripper
simulation_close_tool; %Set do1;
MoveJ(RT_Inicio, 'vmax' , 'fine' , TD_gripper, 'wobj0');
end

 
