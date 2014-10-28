%%%
  VERSION:1
  LANGUAGE:ENGLISH
%%%

MODULE put_in_box
  PERS tooldata gripper:=[TRUE,[[0,0,125],[1,0,0,0]],[0.1,[0,0,100],[1,0,0,0],0,0,0]];
  CONST robtarget pos_inicial:=[[44.29,-501.86,432.17],[0.262525,0.622008,0.673812,-0.300276],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robjoint inicio:=[0,0,0,0,0,0];
  CONST robtarget aproximacion_1:=[[473,-461.6,577],[0.08,0.60337,0.79283,-0.03126],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_rec:=[[34.6,-645.79,342.7],[0.261961,0.652144,0.674197,-0.227034],[-1,2,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_dej1:=[[426.64,-459.99,350],[0.07981,0.603204,0.792983,-0.030902],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_dej2:=[[489.95,-369.78,350],[0.079573,0.60291,0.793227,-0.030978],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_dej3:=[[553.43,-427.83,350],[0.079639,0.60287,0.793244,-0.03114],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  CONST robtarget pos_b_dej4:=[[505.56,-509.74,350],[0.079644,0.60287,0.793239,-0.031255],[-1,1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  VAR num pieza:=0;

  !Reset --> abrir pinza
  !Set --> Cerrar pinza
  PROC Path_10()
    !Moverse a posición base de recogida
    MoveJ pos_inicial,v1000,z100,gripper\WObj:=wobj0;
    COGER_SIGUIENTE;
    !Moverse a posición base de dejada
    MoveJ aproximacion_1,v1000,z100,gripper\WObj:=wobj0;
    METER_EN_CAJA;
  ENDPROC

  PROC COGER_SIGUIENTE()
    !Ahora abrir pinza
    Reset do1;
    WaitTime 0.1;
    !esperar apertura
    !Avanzar a por la siguiente pieza
    MoveL pos_b_rec,v1000,fine,gripper\WObj:=wobj0;
    !Ahora cerrar pinza
    Set do1;
    WaitTime 0.1;
    !esperar cierre
    !Subir por seguridad antes de ir al siguiente punto
    MoveL pos_inicial,v1000,z100,gripper\WObj:=wobj0;
  ENDPROC

  PROC METER_EN_CAJA()
    MoveJ aproximacion_1,v1000,z100,gripper\WObj:=wobj0;
    IF pieza=0 THEN
      !Bajar para meter primera pieza
      MoveL pos_b_dej1,v1000,fine,gripper\WObj:=wobj0;
    ELSEIF pieza=1 THEN
      MoveL pos_b_dej2,v1000,fine,gripper\WObj:=wobj0;
    ELSEIF pieza=2 THEN
      MoveL pos_b_dej3,v1000,fine,gripper\WObj:=wobj0;
    ELSE
      MoveL pos_b_dej4,v1000,fine,gripper\WObj:=wobj0;
    ENDIF
    !Ahora abrir pinza
    Reset do1;
    WaitTime 0.1;
    !Subir por seguridad antes de ir al siguiente punto
    MoveL aproximacion_1,v1000,z100,gripper\WObj:=wobj0;
  ENDPROC

  PROC main()
    !inicializar pieza
    MoveAbsJ [[-18.82,8.68,14.85,179.86,-18.95,-177.13],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v1000,z100,gripper\WObj:=wobj0;
    ConfJ\Off;
    ConfL\Off;
    pieza:=0;
    MoveJ pos_inicial,v1000,z100,gripper\WObj:=wobj0;
    FOR I FROM 1 TO 4 DO
      Path_10;
      pieza:=pieza+1;
    ENDFOR
  ENDPROC
ENDMODULE

