function animacion

global p1 p2 p3 p4 p5 p6 p7 p1aux p2aux

global TD_gripper
global robot

p1=[[1.2254, 1.1375, 1.3],[0.0000, -0.0027, -1.0000, -0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p1aux=[[1.2254, 1.1375, 1.3],[0.0000, -0.0027, -1.0000, -0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p2=[[1.1254, 1.975, 1.2],[0.0000, -0.0027, -1.0000, -0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p2aux=[[1.1254, 1.975, 1.3],[0.0000, -0.0027, -1.0000, -0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p3=[[2.2500, -0.150, 0.080],[0.0000, 1.0000, 0.0000, 0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p4=[[1.5000, -0.150, 0.080],[0.0000, 1.0000, 0.0000, 0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p5=[[1, -1.5, 1.3],[0.0000, -0.6946, 0.7194, 0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p6=[[0.05, -1.5, 1.3],[0.0000, -0.6946, 0.7194, 0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
p7=[[0.05, -1.5, 1.2],[0.0000, -0.6946, 0.7194, 0.0000], [0, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

robot = load_robot('KUKA','KR120_R3200_PA');
robot.T0 = [1 0 0 0;0 1 0 0;0 0 1 1;0 0 0 1];
robot.graphical.draw_axes=0;
robot.equipment{1} = load_robot('equipment/tables','table_big');
robot.equipment{1}.graphical.color = [75 75 75]./255;
robot.equipment{1}.graphical.draw_axes=0;
robot.equipment{1}.T0 = [1 0 0 -0.5;0 1 0 -0.5; 0 0 1 0;0 0 0 1];
robot.tool = load_robot('equipment/end_tools','pallet_gripper');
robot.tool.graphical.draw_axes=0;
robot.equipment{2} = load_robot('equipment','conveyor_belt_2');
robot.equipment{2}.T0 = [0 -1 0 0;1 0 0 2.4; 0 0 1 0; 0 0 0 1];
robot.equipment{2}.graphical.draw_axes=0;
robot.equipment{2}.graphical.color = [1 80 30]./255;
robot.equipment{3} = load_robot('equipment','conveyor_belt_2');
robot.equipment{3}.graphical.color = [1 80 30]./255;
robot.equipment{3}.graphical.draw_axes=0;
robot.equipment{3}.T0 = [1 0 0 0; 0 1 0 -2.4; 0 0 1 0; 0 0 0 1];
robot.equipment{4} = load_robot('equipment/tables','palletizing');
robot.equipment{4}.T0 = [-1 0 0 2.5; 0 -1 0 0; 0 0 1 0; 0 0 0 1];
robot.equipment{4}.graphical.draw_axes=0;
robot.piece = load_robot('equipment/objects','loaded_pallet');
robot.piece.T0 = [0 -1 0 1;1 0 0 2.2;0 0 1 1.23;0 0 0 1];
robot.piece.graphical.draw_axes=0;
robot.piece.graphical.color = [90 70 10]./255;
robot.piece.graphical.draw_transparent=0;
drawrobot3d(robot);


MoveJ(p1,'vmax','z100',TD_gripper, 'wobj0');
MoveJ(p2,'vmax','fine',TD_gripper, 'wobj0');
simulation_grip_piece;
WaitTime(2);
MoveJ(p2aux,'vmax','fine',TD_gripper, 'wobj0');
MoveJ(p1aux,'vmax','z100',TD_gripper, 'wobj0');
MoveJ(p3,'vmax','fine',TD_gripper, 'wobj0');
WaitTime(2);
simulation_release_piece; 
MoveJ(p4,'vmax','fine',TD_gripper, 'wobj0');
WaitTime(20);
MoveJ(p3,'vmax','fine',TD_gripper, 'wobj0');
simulation_grip_piece;
WaitTime(2);
MoveJ(p5,'vmax','z100',TD_gripper, 'wobj0');
MoveJ(p6,'vmax','fine',TD_gripper, 'wobj0');
MoveJ(p7,'vmax','fine',TD_gripper, 'wobj0');
simulation_release_piece;
WaitTime(2);
MoveJ(p5,'vmax','z100',TD_gripper, 'wobj0');
end


