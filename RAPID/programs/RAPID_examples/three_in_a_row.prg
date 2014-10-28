%%%
VERSION:1
LANGUAGE:ENGLISH
%%%

MODULE PRACT


PERS tooldata TD_gripper:=[TRUE,[[0,0,125],[1,0,0,0]],[0.1,[0,0,0.1],[1,0,0,0],0,0,0]];


CONST robtarget RT_pos_ini:=[[44,-501,432],[0.262525,0.622008,0.673812,-0.300276],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

CONST robtarget RT_aprox_rec:=[[30,-600,450],[0.08,0.60337,0.79283,-0.03126],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_pos_rec:=[[35,-629,337],[0.261833,0.652321,0.67413,-0.226869],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

CONST robtarget RT_aprox_dej:=[[470,-460,500],[0.07981,0.603204,0.792983,-0.030902],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget RT_pos_dej:=[[470,-460,450],[0.07981,0.603204,0.792983,-0.030902],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];



VAR NUM VAR_pieza:=0;
!!Reset --> abrir pinza
!!Set --> Cerrar pinza



PROC main()

ConfJ \Off;
ConfL \Off;


!inicializar pieza
VAR_pieza:=0;


MoveJ RT_pos_ini,vmax,z100,TD_gripper\WObj:=wobj0;

FOR I FROM 1 TO 4 DO
!!Moverse a posición de aproximacion de recogida
MoveJ RT_aprox_rec,vmax,z100,TD_gripper\WObj:=wobj0;
COGER_PIEZA;
!!Moverse a posición base de dejada
MoveJ RT_aprox_dej,vmax,z100,TD_gripper\WObj:=wobj0;
METER_EN_CAJA;
VAR_pieza:=VAR_pieza+1;
ENDFOR

ENDPROC




PROC COGER_PIEZA()

!!Ahora abrir pinza
Reset do1; 
Waittime 0.1;
!!Coger
MoveL RT_pos_rec,v500,fine,TD_gripper\WObj:=wobj0;

!!Ahora cerrar pinza
Set do1;

Waittime 0.1;
!!Subir por seguridad antes de ir al siguiente punto
MoveL RT_aprox_rec,v500,z50,TD_gripper\WObj:=wobj0;

ENDPROC

PROC METER_EN_CAJA()

IF VAR_pieza=0 THEN
!Bajar para meter primera pieza
    MoveL Offs(RT_pos_dej,0,0,-105),v1000,fine,TD_gripper\WObj:=wobj0;
ELSEIF VAR_pieza=1 THEN
    MoveL RT_pos_dej,v1000,z100,TD_gripper\WObj:=wobj0;
    MoveL Offs(RT_pos_dej,0,0,-55),v1000,fine,TD_gripper\WObj:=wobj0;
ELSEIF VAR_pieza=2 THEN
    MoveL Offs(RT_pos_dej,0,0,50),v1000,fine,TD_gripper\WObj:=wobj0;
    MoveL Offs(RT_pos_dej,0,0,-5),v1000,fine,TD_gripper\WObj:=wobj0;
ELSE
    MoveL Offs(RT_pos_dej,0,0,100),v1000,fine,TD_gripper\WObj:=wobj0;
    MoveL Offs(RT_pos_dej,0,0,45),v1000,fine,TD_gripper\WObj:=wobj0;
ENDIF

Waittime 0.1;

!Ahora abrir pinza
!yes, release the piece
Reset do1; 


Waittime 0.5;
!Subir por seguridad antes de ir al siguiente punto
MoveL Offs(RT_pos_dej,0,0,105),v1000,z10,TD_gripper\WObj:=wobj0;

ENDPROC


ENDMODULE