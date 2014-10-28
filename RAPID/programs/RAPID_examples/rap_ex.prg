MODULE Rap_ex

    PERS tooldata gripper:=[TRUE,[[0,0,125],[1,0,0,0]],[0.1,[0,0,100],[1,0,0,0],0,0,0]];
	CONST robtarget pos_inicial:=[[701.14,0.0,567.0],[0.5,0,0.8660,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget aproximacion_1:=[[500,0.0,200],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget pos_b_rec:=[[-200,-550,200],[0,0.707106781186547,0.707106781186548,4.32963728535968E-17],[-2,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget pos_b_dej:=[[200,-550,300],[0,0.707181186547,0.707106781186548,4.32963728535968E-17],[-2,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    


	VAR num pieza:=0;
	!Reset --> abrir pinza
	!Set --> Cerrar pinza

PROC Path_10()
	!Moverse a posición base de recogida
	MoveJ pos_b_rec,v1000,z100,gripper\WObj:=wobj0;
	COGER_SIGUIENTE;
	!Moverse a posición base de dejada
	MoveJ pos_b_dej,v1000,fine,gripper\WObj:=wobj0;
	METER_EN_CAJA;
	
ENDPROC
PROC COGER_SIGUIENTE()
	!Ahora abrir pinza
	Reset do1; 
	Waittime 0.1;!esperar apertura
	!Avanzar a por la siguiente pieza
	MoveL Offs(pos_b_rec,0,-50*pieza,-25),v1000,fine,gripper\WObj:=wobj0;
	!Ahora cerrar pinza
	Set do1;
	Waittime 0.1; !esperar cierre
	!Subir por seguridad antes de ir al siguiente punto
	MoveL Offs(pos_b_rec,0,-50*pieza,25),v1000,z10,gripper\WObj:=wobj0;
	
ENDPROC

PROC METER_EN_CAJA()
	
	IF pieza=0 THEN
		!Bajar para meter primera pieza
		MoveL Offs(pos_b_dej,0,0,-105),v1000,fine,gripper\WObj:=wobj0;
	ELSEIF pieza=1 then
		MoveL Offs(pos_b_dej,-55,0,0),v1000,fine,gripper\WObj:=wobj0;
		MoveL Offs(pos_b_dej,-55,0,-105),v1000,fine,gripper\WObj:=wobj0;
	ELSEIF pieza=2 then
		MoveL Offs(pos_b_dej,0,-55,0),v1000,fine,gripper\WObj:=wobj0;
		MoveL Offs(pos_b_dej,0,-55,-105),v1000,fine,gripper\WObj:=wobj0;
	ELSE
		MoveL Offs(pos_b_dej,-55,-55,0),v1000,fine,gripper\WObj:=wobj0;
		MoveL Offs(pos_b_dej,-55,-55,-105),v1000,fine,gripper\WObj:=wobj0;
	ENDIF
	!Ahora abrir pinza
	Reset do1;
	Waittime 0.1;
	!Subir por seguridad antes de ir al siguiente punto
	MoveL Offs(pos_b_dej,0,0,105),v1000,z10,gripper\WObj:=wobj0;
	
ENDPROC

PROC main()
	!inicializar pieza
	pieza:=0;
	MoveJ pos_inicial,v1000,z100,gripper\WObj:=wobj0;
	MoveJ aproximacion_1,v1000,z100,gripper\WObj:=wobj0;
	
	FOR I FROM 1 TO 4 DO
		Path_10;
		pieza :=pieza +1;
	ENDFOR
ENDPROC


ENDMODULE