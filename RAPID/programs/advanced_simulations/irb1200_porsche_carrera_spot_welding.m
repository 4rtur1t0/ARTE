%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Authors:
%       Barreras Almarcha, Héctor
%       Maleno Vizcaíno,  Adrián
%       Tuesta San Miguel, María
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function irb1200_porsche_carrera_spot_welding
global TD_tool0 RT_tp1 RT_tp2 RT_tp3 JV_q0 RT_tp4 RT_tp5 RT_tp6 RT_tp7 robot

robot = load_robot('ABB','IRB1200')
robot.equipment{1} = load_robot('equipment','miscelanea/porsche_carrera');
robot.tool = load_robot('equipment','end_tools/spot_welding');

TD_tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

%initial position
JV_q0=[0 0 0 0 0 0]';
%target points
RT_tp1=[[0.0384, -0.3860, 1.0738],[0.9661, 0.0692, 0.1684, -0.1830], [1, -1, -1, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[0.1078, -0.2019, 1.1489],[0.9661, 0.0692, 0.1684, -0.1830], [1, -2, -1, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[0.1697, -0.2267, 1.1309],[0.9661, 0.0692, 0.1684, -0.1830], [1, -2, -1, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[0.2311 -0.3805 1.466],[0.9661, 0.0692, 0.1684, -0.1830], [1, -2, -1, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[0.1973, -0.1636, 1.1414],[0.9661, 0.0692, 0.1684, -0.1830], [1, -2, 0, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp6=[[0.2287, -0.1743, 1.1299],[0.9661, 0.0692, 0.1684, -0.1830], [1, -2, 0, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp7=[[0.2886 -0.2353 1.491],[0.9661, 0.0692, 0.1684, -0.1830], [1, -2, -1, 4], [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

 MoveAbsJ(JV_q0, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveL(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');

MoveL(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');

MoveL(RT_tp3, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveL(RT_tp4, 'vmax' , 'fine' , TD_tool0, 'wobj0');

MoveL(RT_tp5, 'vmax' , 'fine' , TD_tool0, 'wobj0');


MoveL(RT_tp6, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveL(RT_tp7, 'vmax' , 'fine' , TD_tool0, 'wobj0');





end