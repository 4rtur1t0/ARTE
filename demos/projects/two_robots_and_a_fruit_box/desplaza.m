function [qt1,qt2,qt1g,qt2g,qt11,qt22,qt1f,qt2f]=desplaza(Pf,qi1,qi2,robot,delta_t,coordposicion,coord)
%IGUAL QUE ANTES, SE CALCULA LA POSICION INICIAL Y LA POSICION FINAL
Ti1=directkinematic(robot.robot1,qi1(:,1));
Ti2=directkinematic(robot.robot2,qi2(:,1));
Tf1=Ti1;
Tf1(1:3,4)=Tf1(1:3,4)+Pf;
Tf2=Ti2;
Tf2(1:3,4)=Tf2(1:3,4)+Pf-coordposicion;
z=0.2;%CUANTO LEVANTAN LA CAJA
%PUNTOS INTERMEDIOS
%PUESTO QUE QUEREMOS LEVANTARLA EN LINEA RECTA, GIRARLA Y DESPLAZARLA EN
%LINEA RECTA Y A LA MISMA ALTURA, HAY QUE CALCULAR LOS PUNTOS INTERMEDIOS

T1=Ti1;T1(1:3,4)=T1(1:3,4)+[0;0;z];
T2=Ti2;T2(1:3,4)=T2(1:3,4)+[0;0;z]-coordposicion;%SE RESTA LA COORDENADA DEL SEGUNDO ROBOT PARA
                                                 %QUE TODO ESTE%REFERENCIADO AL SISTEMA
                                                 %DEL PRIMER ROBOT

T12=Tf1;T12(1:3,4)=T12(1:3,4)+[0;0;z];
T22=Tf2;T22(1:3,4)=T22(1:3,4)+[0;0;z];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%POSICIONES ARTICULARES

qf11=inversekinematic(robot.robot1,T1);
qf12=inversekinematic(robot.robot1,T12);%si hay giro se cambia
qf1=inversekinematic(robot.robot1,Tf1);%si hay giro se cambia

qf21=inversekinematic(robot.robot1,T2);
qf22=inversekinematic(robot.robot1,T22);%si hay giro se cambia
qf2=inversekinematic(robot.robot1,Tf2);%si hay giro se cambia

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LEVANTA A CAJA
[qt1,qt2]=calculaq(qi1(:,1),qf11(:,1),qi2(:,1),qf21(:,1),delta_t,1,5,robot);
representa(robot,qt1,qt2,2,coord,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%

%%giro%%
qt1g=interpoladorcirc(robot.robot1,qf11(:,1),pi/2,0.1,1);
qt2g=interpoladorcirc(robot.robot2,qf21(:,1),pi/2,-0.1,2);
representa(robot,qt1g,qt2g,2,coord,1);

T12=directkinematic(robot.robot1,qt1g(:,size(qt1g,2)));
T12(1:3,4)=T12(1:3,4)+Pf+[0.02;0;0];%EL 0.02 ES UN FACTOR DE CORRECCION DEL GIRO
T22=directkinematic(robot.robot1,qt2g(:,size(qt2g,2)));
T22(1:3,4)=T22(1:3,4)+Pf+[-0.02;0;0];
qf22=inversekinematic(robot.robot1,T22);
qf12=inversekinematic(robot.robot1,T12);
Tf1=T12;
Tf1(1:3,4)=Tf1(1:3,4)-[0;0;z];
Tf2=T22;
Tf2(1:3,4)=Tf2(1:3,4)-[0;0;z];
qf2=inversekinematic(robot.robot1,Tf2);
qf1=inversekinematic(robot.robot1,Tf1);
%%%%%%%%%%%%%%%%%%%%%%%%%%
%LA DESPLAZA UNA VEZ GIRADA HASTA LA SIGUIENTE POSICION
[qt11,qt22]=calculaq(qt1g(:,size(qt1g,2)),qf12(:,1),qt2g(:,size(qt2g,2)),qf22(:,1),delta_t,1,5,robot);
representa(robot,qt11,qt22,2,coord,1);

%LA DEJA EN EL SUELO OTRA VEZ
[qt1f,qt2f]=calculaq(qt11(:,size(qt11,2)),qf1(:,1),qt22(:,size(qt22,2)),qf2(:,1),delta_t,1,5,robot);
representa(robot,qt1f,qt2f,2,coord,1);


end