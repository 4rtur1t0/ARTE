%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Authors: EMILIO GÓMEZ, GUILLERMO MAZÓN y LAURA RODRÍGUEZ.
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function staubli_tx90xl_simulation()
global robot
robot = load_robot('STAUBLI','TX90XL')
robot.equipment{1} = load_robot('equipment','tables/table_two_areas');
robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
robot.piece{1}=load_robot('equipment','cylinders/cylinder_tiny');

RT_tp1=[[0.8, 0, 0.215],[0.0177, 0.9946, -0.1019, -0.0007], [0, -1, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[0.7903, 0.1533, 0.6155],[0.0007, 0.1019, 0.9946, 0.0177], [0, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[-0.4663, -0.6475, 0.6700],[0.0245, 0.9620, 0.2713, -0.0173], [-2, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3i=[[-0.4663, -0.6475, 0.6],[0.0245, 0.9620, 0.2713, -0.0173], [-2, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[-0.6, -0.75, 0.4570],[0.0033, -0.2559, 0.9667, -0.0086], [-2, 0, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4i=[[-0.6, -0.75, 1.2],[0.0033, -0.2559, 0.9667, -0.0086], [-2, 0, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[0.02, -0.8, 0.4237],[0.0054, 0.5413, 0.8408, 0.0095], [-1, 0, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5i=[[0.02, -0.8, 0.9],[0.0054, 0.5413, 0.8408, 0.0095], [-1, 0, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5ii=[[0.5, -0.3, 0.6],[0.0054, 0.5413, 0.8408, 0.0095], [-1, 0, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tpi=[[0.0123, 0.0854, 1.8779],[0.0000, 0.0000, -0.0000, 1.0000], [0, 0, 0, 3], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2i=[[1.1241, -0.7430, 0.4831],[0.1704, 0.8046, -0.2754, 0.4977], [-1, -1, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

robot.piece{1}.T0= [1 0 0 -0.1;
                        0 1 0 -0.5;
                        0 0 1 0.2;
                        0 0 0 1]; 



simulation_open_tool;
MoveJ(RT_tpi, 'vmax' , 'fine' , tool0, 'woj0');
MoveJ(RT_tp1, 'vmax' , 'fine' , tool0, 'wobj0');
simulation_close_tool;
simulation_grip_piece;
MoveJ(RT_tp5ii, 'vmax' , 'fine' , tool0, 'wobj0'); 
MoveJ(RT_tp3, 'vmax' , 'fine' , tool0, 'wobj0'); 
MoveJ(RT_tp4, 'vmax' , 'fine' , tool0, 'wobj0'); 
simulation_open_tool;
simulation_release_piece;
MoveJ(RT_tp4i, 'vmax' , 'fine' ,tool0, 'wobj0');
WaitTime(2);
MoveJ(RT_tp4, 'vmax' , 'fine' , tool0, 'wobj0');
simulation_close_tool;
simulation_grip_piece;
MoveJ(RT_tp3i, 'vmax' , 'fine' , tool0, 'wobj0'); 
MoveJ(RT_tp5i, 'vmax' , 'fine' , tool0, 'wobj0');
MoveJ(RT_tp5, 'vmax' , 'fine' , tool0, 'wobj0');
simulation_open_tool;
simulation_release_piece;
MoveJ(RT_tp5i, 'vmax' , 'fine' , tool0, 'wobj0');
MoveJ(RT_tp5ii, 'vmax' , 'fine' , tool0, 'wobj0');
WaitTime(1);
MoveJ(RT_tp1, 'vmax' , 'fine' , tool0, 'wobj0');
