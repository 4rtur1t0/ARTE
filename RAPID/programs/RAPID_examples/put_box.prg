%%%
  VERSION:1
  LANGUAGE:ENGLISH
%%%

MODULE put_box
  PERS tooldata gripper:=[TRUE,[[0,0,125],[1,0,0,0]],[0.1,[0,0,100],[1,0,0,0],0,0,0]];
  CONST robtarget pos_inicial:=[[44.29,-501.86,432.17],[0.262525,0.622008,0.673812,-0.300276],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robjoint inicio:=[0,0,0,0,0,0];
  CONST robtarget aproximacion_1:=[[473,-461.6,577],[0.08,0.60337,0.79283,-0.03126],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_rec:=[[34.95,-629.79,337.37],[0.261833,0.652321,0.67413,-0.226869],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_dej:=[[470,-460,450],[0.07981,0.603204,0.792983,-0.030902],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  VAR num pieza:=0;

  !Reset --> abrir pinza
  !Set --> Cerrar pinza
  PROC Path_10()
    COGER_SIGUIENTE;
    !Moverse a posición base de dejada
    MoveJ pos_b_dej,v1000,z10,gripper\WObj:=wobj0;
    METER_EN_CAJA;
  ENDPROC

  PROC COGER_SIGUIENTE()
    !Ahora abrir pinza
    Reset do1;
    WaitTime 0.1;
    !esperar apertura
    !Avanzar a por la siguiente pieza    
    !Moverse a posición base de recogida
    MoveJ Offs(pos_b_rec,50,50,150),v500,z15,gripper\WObj:=wobj0;
    MoveL pos_b_rec,v500,fine,gripper\WObj:=wobj0;
    
    !Ahora cerrar pinza
    Set do1;
    WaitTime 0.1;
    !esperar cierre
    !Subir por seguridad antes de ir al siguiente punto
    MoveL Offs(pos_b_rec,0,0,50),v1000,z10,gripper\WObj:=wobj0;
    MoveL Offs(pos_b_rec,0,50,50),v1000,z10,gripper\WObj:=wobj0;

  ENDPROC

  PROC METER_EN_CAJA()
    IF pieza=0 THEN
      !Bajar para meter primera pieza
      MoveL Offs(pos_b_dej,0,0,-105),v1000,fine,gripper\WObj:=wobj0;
    ELSEIF pieza=1 THEN
      MoveL Offs(pos_b_dej,45,75,0),v1000,z10,gripper\WObj:=wobj0;
      MoveL Offs(pos_b_dej,45,75,-105),v1000,fine,gripper\WObj:=wobj0;
    ELSEIF pieza=2 THEN
      MoveL Offs(pos_b_dej,105,65,0),v1000,z10,gripper\WObj:=wobj0;
      MoveL Offs(pos_b_dej,105,65,-105),v1000,fine,gripper\WObj:=wobj0;
    ELSE
      MoveL Offs(pos_b_dej,55,-25,0),v1000,z10,gripper\WObj:=wobj0;
      MoveL Offs(pos_b_dej,55,-25,-105),v1000,fine,gripper\WObj:=wobj0;
    ENDIF
    !Ahora abrir pinza
    Reset do1;
    WaitTime 0.1;
    !Subir por seguridad antes de ir al siguiente punto
    MoveL Offs(pos_b_dej,0,0,105),v1000,z10,gripper\WObj:=wobj0;
  ENDPROC

  PROC main()
    ConfJ\Off;
    ConfL\Off;
    !inicializar pieza
    pieza:=0;
    MoveAbsJ [[0,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v300,z100,gripper\WObj:=wobj0;
    FOR I FROM 1 TO 4 DO
      Path_10;
      pieza:=pieza+1;
    ENDFOR
  ENDPROC
ENDMODULE

