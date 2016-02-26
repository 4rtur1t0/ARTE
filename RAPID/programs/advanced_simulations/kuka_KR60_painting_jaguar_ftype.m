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

function painting2

global TD_tool0 RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8 RT_tp9 RT_tp10 RT_tp11 RT_tp12 RT_tp13 RT_tp14 RT_tp15 RT_tp16


TD_tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

RT_tp1=[[1.3400, -0.0000, 1.8100],[0.7006, -0.0954, 0.7006, -0.0954], [0, 0, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[1.1648, 0.6617, 1.8206],[0.7233, -0.2626, 0.6324, 0.0894], [-1, 0, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[1.4924, 0.9803, 1.1556],[0.5816, -0.3339, 0.7362, 0.0911], [-1, 0, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[1.5020, 0.8533, 1.3436],[0.6364, -0.2942, 0.7086, 0.0787], [-1, 0, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[1.5207, 0.6786, 1.4836],[0.6784, -0.2480, 0.6898, 0.0507], [-1, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp6=[[1.6593, 0.3815, 1.4044],[0.6580, -0.1850, 0.7298, -0.0146], [-1, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp7=[[1.6990, 0.1099, 1.4044],[0.6547, -0.1255, 0.7423, -0.0677], [-1, 0, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp8=[[1.6707, -0.3278, 1.4044],[0.6406, -0.0289, 0.7523, -0.1514], [0, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp9=[[1.6064, -0.8202, 1.1348],[0.5463, 0.0815, 0.8060, -0.2127], [0, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp10=[[1.4123, -1.1863, 0.9392],[0.5177, 0.1723, 0.7925, -0.2725], [0, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp11=[[1.2529, -1.3639, 0.9736],[0.5680, 0.2052, 0.7172, -0.3476], [0, 0, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp12=[[1.2075, -0.7363, 1.1201],[0.5774, 0.1073, 0.7697, -0.2503], [0, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp13=[[1.2048, -0.5831, 1.2640],[0.5698, 0.0714, 0.7900, -0.2149], [0, 0, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp14=[[1.2754, -0.1696, 1.3640],[0.6362, -0.0526, 0.7587, -0.1299], [0, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp15=[[1.2481, 0.3127, 1.3640],[0.6493, -0.1940, 0.7354, -0.0081], [-1, 0, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp16=[[1.1268, 0.6211, 1.3640],[0.6449, -0.2871, 0.7042, 0.0756], [-1, -1, 1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
main
end

function main()

global TD_tool0 RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8 RT_tp9 RT_tp10 RT_tp11 RT_tp12 RT_tp13 RT_tp14 RT_tp15 RT_tp16

 MoveL(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp3, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp4, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp5, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp6, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp7, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp8, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp9, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp10, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp11, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp12, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp13, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp14, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp15, 'vmax' , 'fine' , TD_tool0, 'wobj0');
 MoveL(RT_tp16, 'vmax' , 'fine' , TD_tool0, 'wobj0');

end
