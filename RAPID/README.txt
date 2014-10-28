In the programs directory you can find Matlab scripts that allow to simulate the robot.
To do this, first load a robot:

>>robot=load_robot('abb', 'irb140')

The examples have been tested with the ABB IRB140 robot.

The scripts can be translated to RAPID language with ease, just call:

>> matlab2RAPID

And select your .m file. The matlab2RAPID function will generate a .prg file at the same directory.
The name of the .prg file and Module name are shortened, since old controllers 
do not accept long character names.

Since the Matlab language does not declare data types, some conventions need to be
considered to enable a correct translation between the Matlab and RAPID code.

a) Target points must be declared with names starting with RT_
b) Tool data should be named starting with TD_
c) A main function should be named and called from the main script.
d) Target points, tool data and other variables should be declared global. This step
is only needed for an easy simulation.
e) Functions should start with the keyword function and end with the end keyword.
f) Please, copy and edit any of the existing scripts to develop your simulation easily.


For example, the file basic_robot_program.m contains:

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function basic_robot_program

global RT_tp1 RT_tp2 RT_tp3 RT_tp4 TD_gripper

TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];
RT_tp1=[[0.541,0.1171,0.713],[0.727812,-0.115363,0.676004,-0.000468],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp2=[[0.515,-0.200,0.712],[0.7071,0,0.7071,0],[-1,-2,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp3=[[0.515,0,0.912],[0.7071,0,0.7071,0],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_tp4=[[0.515,0,0.512],[0.7071,0,0.7071,0],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

main
end

function main

global RT_tp1 RT_tp2 RT_tp3 RT_tp4 TD_gripper

MoveJ(RT_tp1, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveJ(RT_tp2, 'vmax' , 'fine' , TD_gripper, 'wobj0');
MoveJ(RT_tp3,'vmax','fine', TD_gripper, 'wobj0');
MoveL(RT_tp4,'vmax','fine', TD_gripper, 'wobj0');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

And is translated to RAPID as:

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


Finally, there exists some example code under the directory programs/RAPID_example
that consist of full examples of RAPID programs.

 