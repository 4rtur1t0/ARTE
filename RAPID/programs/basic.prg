%%%
VERSION:1
LANGUAGE:ENGLISH
%%%

MODULE BASIC

PERS tooldata TD_gripper:=[TRUE,[[0,0,125],[1,0,0,0]],[0.1,[0,0,0.1],[1,0,0,0],0,0,0]];
CONST robtarget RT_tp1:=[[541,117,713],[0.727812,-0.115363,0.676004,-0.000468],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_tp2:=[[515,-200,712],[0.7071,0,0.7071,0],[-1,-2,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_tp3:=[[515,0,912],[0.7071,0,0.7071,0],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_tp4:=[[515,0,512],[0.7071,0,0.7071,0],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];


PROC main()

ConfJ \Off;
ConfL \Off;


MoveJ RT_tp1,vmax,fine,TD_gripper\WObj:=wobj0;
MoveJ RT_tp2,vmax,fine,TD_gripper\WObj:=wobj0;
MoveJ RT_tp3,vmax,fine,TD_gripper\WObj:=wobj0;
MoveL RT_tp4,vmax,fine,TD_gripper\WObj:=wobj0;
ENDPROC

ENDMODULE