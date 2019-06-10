%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: DanielCaparrós Hernández
%           Ramón David Sánchez Munuera
%   email: daniel.caparros@alu.umh.es 
%          ramon.sanchez@alu.umh.es
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function kuka_KR10_helmet_painting_simulation
global TD_pistola robot RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8 RT_tp9 


robot = load_robot('KUKA','KR10_R900');
robot.equipment{1} = load_robot('equipment','tables/table_extended');
robot.tool= load_robot('equipment','end_tools/paint_gun');
robot.piece{1}=load_robot('equipment','miscelanea/helmet');
robot.piece{1}.T0=[1 0 0 -0.4;
                0 1 0 -0.4;
                0 0 1 0.2;
                0 0 0 1];

drawrobot3d(robot, robot.q);

RT_tp1=[[-0.06,-0.5,0.7],[0.5247,0.4740,0.5247,-0.4740],[-2,-1,0,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
RT_tp2=[[0.0, 0.0, .55],[0.5222, -0.4307, -0.3463, 0.6495], [-1, -2, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[0.0, 0.0, .55],[0.3338, 0.6595, 0.5303, -0.4152], [-2, -1, -1, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[-0.3, -0.2023, 0.55],[0.3875, 0.8161, 0.1423, 0.4045], [-2, 0, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[-0.1, -0.6, 0.55],[0.0743, -0.7086, -0.6387, -0.2905], [-2, 0, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp6=[[0.0272,-0.6098,0.5540],[0.3058,-0.5869,-0.7447,-0.0864],[-1,-1,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
RT_tp7=[[0.14, -0.16, 0.6],[0.0274, 0.2758, 0.7261, -0.6292], [-1, -1, -1, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp8=[[0.0, -0.42, 0.7],[0.0468, -0.7008, -0.7118, -0.0068], [-1, -1, 0, 0], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp9=[[0.53, 0.0, 0.7],[0.7071, -0.0000, 0.7071, 0.0000], [0, 0, -1, 1], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

TD_pistola=[1,[[0,0,.1],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

main()

end

function main()

global TD_pistola RT_tp1 RT_tp3 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8 RT_tp9 robot

adjust_view(robot)

% move helmet to paint position
for i=1:8,
    %just update the X position in the T0 matrix corresponding to the piece
     robot.piece{1}.T0(1,4)=robot.piece{1}.T0(1,4)+i*0.01
     drawrobot3d(robot, robot.q);
     pause(0.2);
end

drawrobot3d(robot, robot.q);

MoveJ(RT_tp1, 'vmax' , 'fine' , TD_pistola, 'wobj0');
%MoveJ(RT_tp3, 'vmax' , 'fine' , TD_pistola, 'wobj0');
MoveJ(RT_tp4, 'vmax' , 'fine' , TD_pistola, 'wobj0');
MoveJ(RT_tp5, 'vmax' , 'fine' , TD_pistola, 'wobj0');
MoveL(RT_tp6, 'vmax' , 'fine' , TD_pistola, 'wobj0');
MoveJ(RT_tp7, 'vmax' , 'fine' , TD_pistola, 'wobj0');
MoveJ(RT_tp8, 'vmax' , 'fine' , TD_pistola, 'wobj0');

robot.piece{1}.graphical.color = [40 45 40]./255;
drawrobot3d(robot, robot.q);

MoveJ(RT_tp9, 'vmax' , 'fine' , TD_pistola, 'wobj0');

%robot.piece.graphical.color = [40 45 40]./255;

%the helmet resumes its movement along the conveyor belt    
for i=1:8   
     robot.piece{1}.T0(1,4)=robot.piece{1}.T0(1,4)+i*0.05
     drawrobot3d(robot, robot.q)
     pause(0.2);    
end


end