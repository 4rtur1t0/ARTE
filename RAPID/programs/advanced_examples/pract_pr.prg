%%%
VERSION:1
LANGUAGE:ENGLISH
%%%

MODULE PRAC_pr
!init the position of the piece at the beginning of the simulation
!robot.piece.T0(1:3,4)=[-0.1 -0.5 0.2]';
!robot.tool.piece_gripped=0;


!define the tool
!In RAPID this is done by means of the tooldata structure
PERS tooldata TD_gripper:=[TRUE,[[0,0,125],[1,0,0,0]],[0.1,[0,0,0.1],[1,0,0,0],0,0,0]];

!define target points FOR SIMULATION

!replace the above points TO PROGRAM THE REAL ROBOT.
CONST robtarget RT_initial:=[[500,-400,500],[0.427269,0.289111,0.84406,-0.14635],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_approach1:=[[-100,-500,400],[0,0.70711,0.70711,0],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_grip:=[[-97.01,-493.4,246.8],[1.5E-05,-0.707114,-0.7071,-2E-06],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_approach2:=[[532.11,-422.58,368.01],[6.1E-05,-0.707117,-0.707097,1.5E-05],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_release:=[[503.36,-442.47,356.87],[0.000279,0.707036,0.707178,0.000235],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];



PROC main()

ConfJ \Off;
ConfL \Off;

!close the tool
Set do1;

!move to the initial point
MoveJ RT_initial,vmax,fine,TD_gripper\WObj:=wobj0;
! Now open the tool
ReSet do1; 

!Move to the approaching point
MoveJ RT_approach1,vmax,fine,TD_gripper\WObj:=wobj0;
!Now, go down to the grabbing target point and
MoveL RT_grip,vmax,fine,TD_gripper\WObj:=wobj0;
!and close the tool and grip the piece. These two functions
! must be called to simulate that the gripper has the piece grabbed and the
!tool is closed
Set do1;

!Now go to the same approach point so that collisions with the table are
!avoided
MoveL RT_approach1,vmax,fine,TD_gripper\WObj:=wobj0;

!Move to the approach next to the packaging area
MoveJ RT_approach2,vmax,fine,TD_gripper\WObj:=wobj0;
!go down to the release point inside the box
MoveL RT_release,vmax,fine,TD_gripper\WObj:=wobj0;
!yes, release the piece
ReSet do1; 

!now, go up
MoveL RT_approach2,vmax,fine,TD_gripper\WObj:=wobj0;

!Now, go back to the initial point
MoveJ RT_initial,vmax,fine,TD_gripper\WObj:=wobj0;

ENDPROC

ENDMODULE