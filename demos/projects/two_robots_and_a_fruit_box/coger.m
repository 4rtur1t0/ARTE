function [qf1,qf2,qt1,qt2,coord1]=coger(delta_t,robot,coordposicion)
%IGUALAMOS LA POSICION INICIAL A LA ACTUAL DEL ROBOT
qi1=robot.robot1.q;
qi2=robot.robot2.q;
%SE CALCULA DONDE ESTA LA CAJA
Tc1=directkinematic(robot.robot1.equipment{1},robot.robot1.equipment{1}.q);
Tc2=Tc1;

[coord1,R1]=calcgrip(robot.robot1.T0,Tc1);%CALCULA EL AGARRE
%LA FUNCION DEVUELVE DONDE TIENE QUE COGER LA CAJA EL PRIMER ROBOT,
%TENIENDO EN CUENTA SU TAMAÑO
%Y PARA EL SEGUNDO ES JUSTO EL OPUESTO
coord2(1:2,1)=-coord1(1:2,1);
coord2(3,1)=coord1(3,1);
R2=-R1;
Tc1(1:3,4)=Tc1(1:3,4)+coord1;
Tc2(1:3,4)=Tc2(1:3,4)+coord2+[0;0.01;0];
Tf1=Tc1;
Tf2=Tc2;
Tf2(1:3,4)=Tf2(1:3,4)-coordposicion;
Tf1(1:3,1:3)=R1;
Tf2(1:3,1:3)=R2;
%----------------------------------------
%CALCULAMOS LAS POSICIONES FINALES
qf1=inversekinematic(robot.robot1,Tf1);
qf2=inversekinematic(robot.robot2,Tf2);
%-----------------------------------------
%LLAMAMOS A LA FUNCION QUE USA UN INTERPOLADOR U OTRO EN FUNCION DE UN
%PARAMETRO, EN ESTE CASO ES EL 3, INTERPOLADOR DE 3ER ORDEN

[qt1,qt2]=calculaq(qi1,qf1(:,1),qi2,qf2(:,1),delta_t,3,10,robot);

%REPRESENTA EL ROBOT
representa(robot,qt1,qt2,1,coord1,1);



end