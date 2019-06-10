%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Courtesy of: Manuel Esquer Cerezo, 
%              Arturo Samper Rodríguez and
%              David Zambrana Vinaroz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function staubli_tx200_simulation
global robot TD_gripper RT_init RT_aprox1 RT_cog1 RT_aprox2 RT_cog2 RT_aprox11 RT_med1 RT_med2 RT_med3 RT_med4 RT_med5 


robot = load_robot('STAUBLI','TX200');
robot.tool= load_robot('equipment','end_tools/vacuum_1');
robot.equipment{1}=load_robot('equipment','pallet');
robot.piece{1} = load_robot('equipment','box_big');
robot.graphical.draw_axes=0;
robot.piece{1}.graphical.draw_axes=0;
robot.equipment{1}.graphical.draw_axes=0;
robot.tool.graphical.draw_axes=0;
robot.graphical.draw_transparent=0;


RT_aprox1=[[-1.6837, 0.0000, 0.5328],[0.0000, -0.707, 0.0000, 0.707], [-1, -1, -2, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_cog1=[[-2.0131, 0.0000, 0.5258],[0.0000, -0.707, -0.0000, 0.707], [-1, -1, -2, 6], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_init=[[0.2570, -0.0000, 3.011],[1.0000, 0.0000, 0.0000, -0.0000], [0, 0, 0, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_aprox11=[[-2.0131, 0.0000, 0.6050],[0.000, -0.7061, 0.000, 0.7081], [-1, -1, -2, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_med4=[[0.4705, -1.6198, 0.9909],[0.5663, 0.4240, 0.5647, -0.4252], [1, -2, -1, 5], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_aprox2=[[2, 0, .8],[0.707, 0, 0.707, 0], [1, -2, -1, 5], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_cog2=[[2.1031, -0.0000, 0.6488],[0.707, 0, 0.707, 0], [2, -2, -1, 6], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

robot.equipment{1}.T0(1:3,4)=[2.6 0 0]';
robot.piece{1}.T0(1:3,4)=[-3 -.5 0]';

TD_gripper=[1,[[0,0,0.425],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

main
end

function main()

global robot TD_gripper RT_init RT_aprox1 RT_cog1 RT_aprox2 RT_cog2 RT_aprox11 RT_med1 RT_med2 RT_med3 RT_med4 RT_med5 

simulation_open_tool;

MoveJ(RT_init, 'vmax' , 'fine' , TD_gripper);
MoveJ(RT_aprox1, 'vmax' , 'fine' , TD_gripper);
MoveL(RT_cog1, 'vmax' , 'fine' , TD_gripper);

simulation_close_tool; %Set do1;
simulation_grip_piece;

MoveL(RT_aprox1, 'vmax' , 'fine' , TD_gripper);

MoveJ(RT_med4, 'vmax' , 'fine' , TD_gripper);

MoveJ(RT_aprox2, 'vmax' , 'fine' , TD_gripper);
MoveL(RT_cog2, 'vmax' , 'fine' , TD_gripper);

simulation_open_tool; %Reset do1; 
simulation_release_piece;

MoveL(RT_aprox2, 'vmax' , 'fine' , TD_gripper);
MoveJ(RT_init, 'vmax' , 'fine' , TD_gripper);

end
