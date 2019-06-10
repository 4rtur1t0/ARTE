function simulation()
global robot
%Robot y equipos utilizados
robot = load_robot('KUKA','KR30_3')
robot.equipment{1} = load_robot('equipment','transportadoras');
robot.tool= load_robot('equipment','end_tools/parallel_gripper_1');
robot.piece{1}=load_robot('equipment','regalo');


adjust_view(robot);
% robot.piece.T0(1:3,4)=[0.2 -0.5 0.2]';

global tool0 RT_tpini RT_tpgripper RT_tp1 RT_tp2 RT_tp3 RT_tpabrir1 RT_tpabrir2 RT_tpcoger1 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8 RT_tp9

tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];


%Target points para la simulación
RT_tpini=[[1.4600, -0.0000, 1.8100],[0.0000, 0.7071, 0.0000, 0.7071], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tpgripper=[[1.3270, 0.0000, 1.5660],[0.0000, 1.0000, 0.0000, 0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp1=[[1.3000, 0.0000, 1.5660],[0.0000, 1.0000, 0.0000, -0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[1.3000, 0.0000, 0.9000],[0.0000, 1.0000, 0.0000, -0.0000], [0, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[1.41000, -0.4000, 0.9000],[0.0000, 1.0000, 0.0000, 0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tpabrir1=[[1.4100, -0.4000, 0.8800],[0.0000, 1.0000, -0.0000, 0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tpcoger1=[[1.4100, -0.4000, 0.8300],[0.0000, 1.0000, -0.0000, 0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[1.4100, -0.4000, 1.2200],[0.0000, 1.0000, -0.0000, -0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[0.8472, -1.1960, 1.2200],[0.0000, 0.9431, -0.3326, -0.0000], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp6=[[-0.0432, -1.4650, 1.2200],[0.0000, 0.7888, -0.6146, 0.0000], [-2, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp7=[[-1.3434, -0.5859, 1.2200],[0.0000, 1.0000, -0.0000, 0.0000], [-2, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tpabrir2=[[-1.55, -0.5860, 0.9000],[0.0000, 1.0000, -0.0000, 0.0000], [-2, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp8=[[-1.65, -0.5860, 1.1000],[0.0000, 1.0000, -0.0000, 0.0000], [-2, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp9=[[-0.0260, -1.4600, 1.8100],[0.0000, 0.7071, 0.0000, 0.7071], [-1, -2, 1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];


    for i=-2.9:0.2:-0.3
    robot.piece{1}.T0 = [1 0 0 1.3;
             0 1 0 i;
             0 0 1 0.45;
             0 0 0 1];
    drawrobot3d(robot,robot.q);
    adjust_view(robot);
    end
    
MoveJ(RT_tpini, 'vmax' , 'fine' , tool0, 'wobj0');
MoveJ(RT_tpabrir1, 'vmax' , 'fine' , tool0, 'wobj0'); 

simulation_open_tool;
MoveL(RT_tpcoger1, 'vmax' , 'fine' , tool0, 'wobj0');
simulation_close_tool;
simulation_grip_piece;

MoveL(RT_tp4, 'vmax' , 'fine' , tool0, 'wobj0');
MoveL(RT_tp5, 'vmax' , 'fine' , tool0, 'wobj0'); 
MoveL(RT_tp6, 'vmax' , 'fine' , tool0, 'wobj0'); 
MoveL(RT_tp7, 'vmax' , 'fine' , tool0, 'wobj0'); 

MoveL(RT_tpabrir2, 'vmax' , 'fine' ,tool0, 'wobj0');
simulation_open_tool;
simulation_release_piece;

MoveL(RT_tp8, 'vmax' , 'fine' , tool0, 'wobj0');
MoveJ(RT_tpini, 'vmax' , 'fine' , tool0, 'wobj0');


    for i=-0.5860:0.2:2.8
    robot.piece{1}.T0 = [1 0 0 -1.65;
             0 1 0 i;
             0 0 1 0.45;
             0 0 0 1];
    drawrobot3d(robot,robot.q);
    adjust_view(robot);
    end

end

