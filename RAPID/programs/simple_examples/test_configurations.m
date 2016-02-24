% IN ORDER TO SIMULATE THE PROGRAM:
%   A) FIRST, LOAD A ROBOT
%       robot = load_robot('abb','irb140');
%   
% The example presents the robot achieving the same position and
% orientation in space with eight different possible configurations.
% 
% This program is equivalent to test_config.prg. Just execute matlab2RAPID
% and select test_configurations.m to obtain it. You may delete some Matlab
% expressions that cannot be converted to RAPID.
%
% Have a look to this video that represents the execution in Matlab and the
% execution in the real controller:
%
% http://www.youtube.com/watch?v=4HepUzuVyUs&list=PLClKgnzRFYe72qDYmj5CRpR9ICNnQehup&index=27
%
function test_configurations

%global JT_q1 JT_q2 JT_q3 JT_q4 JT_q5 JT_q6 JT_q7 JT_q8
global RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8 TD_tool0 robot


% T =[0.0    0.0    1.0    0.5150;
%    0.0   1.0   0.0    0.2000
%    -1.0   0.0   0.0    0.7120
%     0         0         0    1.0];
%     
% q_inv=inversekinematic(robot, T);

%definition of the end effector.
TD_tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

%JT_q1=[[23.9625,6.7721,-7.1508,90.852,-23.9653,-90.9324],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]*pi/180;

RT_tp1=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[0,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[0,1,-2,3],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[0,-1,0,2],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp5=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[-2,-1,-2,5],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp6=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[-2,1,0,4],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp7=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[-2,-1,-2,7],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp8=[[0.515,0.200,0.712],[0.7071,0,0.7071,0],[-2,1,0,6],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

main
end

function main
%global JT_q1 JT_q2 JT_q3 JT_q4 JT_q5 JT_q6 JT_q7 JT_q8
global RT_tp1 RT_tp2 RT_tp3 RT_tp4 RT_tp5 RT_tp6 RT_tp7 RT_tp8 TD_tool0 robot


MoveJ(RT_tp1, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp2, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp3, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp4, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp5, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp6, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp7, 'vmax' , 'fine' , TD_tool0, 'wobj0');
MoveJ(RT_tp8, 'vmax' , 'fine' , TD_tool0, 'wobj0');
  

end