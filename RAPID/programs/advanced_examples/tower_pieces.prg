%%%
  VERSION:1
  LANGUAGE:ENGLISH
%%%

MODULE put_box
  PERS tooldata gripper:=[TRUE,[[0,0,125],[1,0,0,0]],[0.1,[0,0,100],[1,0,0,0],0,0,0]];
  CONST robtarget pos_inicial:=[[44.29,-501.86,432.17],[0.262525,0.622008,0.673812,-0.300276],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robjoint inicio:=[0,0,0,0,0,0];
  CONST robtarget aproximacion_1:=[[500,-461.6,577],[0.08,0.60337,0.79283,-0.03126],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_rec:=[[-20.24,-620.57,400.56],[0.247301,0.614509,0.23458,0.711473],[-2,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_rec1:=[[-86.08,-649.7,281.46],[0.255916,0.633214,0.229374,0.693495],[-2,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_dej:=[[663.48,79.36,290.0],[0.343109,-0.632434,-0.332522,-0.609699],[0,1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  VAR num pieza:=0;

  !Reset --> abrir pinza
  !Set --> Cerrar pinza
  PROC Path_prin()
    COGER_SIGUIENTE;
    MoveAbsJ [[-30,-10,-10,20,20,20],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v500,z100,gripper\WObj:=wobj0;
    MoveL Offs(pos_b_dej,0,0,300),v1000,z10,gripper\WObj:=wobj0;
    METER_EN_CAJA;
    MoveAbsJ [[-30,-10,-10,20,20,20],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v500,z100,gripper\WObj:=wobj0;

  ENDPROC

  PROC COGER_SIGUIENTE()

    !Ahora abrir pinza
    Reset do2;
    WaitTime 0.1;
    !esperar apertura
    !Avanzar a por la siguiente pieza    
    !Moverse a posición base de recogida
    MoveJ pos_b_rec,vmax,z15,gripper\WObj:=wobj0;
    MoveL pos_b_rec1,v500,fine,gripper\WObj:=wobj0;
    !Ahora cerrar pinza
    Set do2;
    WaitTime 0.1;
    !esperar cierre
    !Subir por seguridad antes de ir al siguiente punto
    MoveL pos_b_rec,v1000,z10,gripper\WObj:=wobj0;
    !MoveL Offs(pos_b_rec,0,50,50),v1000,z10,gripper\WObj:=wobj0;
  ENDPROC

  PROC METER_EN_CAJA()
    IF pieza=0 THEN
      !Bajar para meter primera pieza
      MoveL Offs(pos_b_dej,0,0,0),v1000,fine,gripper\WObj:=wobj0;
    ELSEIF pieza=1 THEN
      MoveL Offs(pos_b_dej,0,0,50),v1000,fine,gripper\WObj:=wobj0;
      
    ELSEIF pieza=2 THEN
      MoveL Offs(pos_b_dej,0,0,100),v1000,fine,gripper\WObj:=wobj0;
    ELSE
       MoveL Offs(pos_b_dej,0,0,150),v1000,fine,gripper\WObj:=wobj0;
    ENDIF
    !Ahora abrir pinza
    Reset do2;
    WaitTime 0.1;
    !Subir por seguridad antes de ir al siguiente punto
    MoveL Offs(pos_b_dej,0,0,300),v1000,z10,gripper\WObj:=wobj0;
  ENDPROC

  PROC main()
    ConfJ\Off;
    ConfL\Off;
    !inicializar pieza
    pieza:=0;
    MoveAbsJ [[-30,10,10,20,20,20],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v500,z100,gripper\WObj:=wobj0;
    FOR I FROM 1 TO 4 DO
      Path_prin;
      pieza:=pieza+1;
    ENDFOR
	reset do2;
  ENDPROC
ENDMODULE

