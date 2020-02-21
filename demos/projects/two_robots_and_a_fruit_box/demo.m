%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CARGA LOS DOS ROBOTS Y LA CAJA
 robot1=load_robot('MOTOMAN','GP7');
 robot2=load_robot('MOTOMAN','GP7');
   coordposicion=[1.2; 0; 0];%INDICA LA POSICION DEL SEGUNDO ROBOT
  robot2.T0(1:3,4)=robot1.T0(1:3,4)+coordposicion;
 
 robot.robot1=robot1;
 robot.robot2=robot2;
 robot.robot1.graphical.draw_axes=0;
 robot.robot2.graphical.draw_axes=0;
 robot.nserial=2;
 robot.robot1.equipment{1}=load_robot('equipment/objects','fruit_box');
 Tcaja=robot.robot1.equipment{1}.T0;
  Pc=[0.5;0;0];
  Tcaja(1:3,4)=Pc;%indicamos donde está la caja
  
  robot.robot1.equipment{1}.T0=Tcaja;
 %----------------------------------------------
 drawrobot3d(robot.robot1,robot.robot1.T0)
 drawrobot3d(robot.robot2,robot.robot2.T0,1)
 adjust_view(robot.robot1)
%INDICAMOS EL PASO
delta_t=0.1;%CAMBIAR A 0.1 SINO EL VECTOR TIEMPO NO FUNCIONA YA QUE LA 
            %FUNCION DE GIRO TIENE UN DELTA_T PROPIO DE 0.1
%---------------------------------
%DESPLAZA LOS DOS ROBOTS DE LA POSICION EN LA QUE ESTEN A LA CAJA
[qf1,qf2,qt11,qt12,coord]=coger(delta_t,robot,coordposicion);
%--------------------------
%INDICAMOS EL PUNTO FINAL, INICIAL Y LA DISTANCIA A LA QUE SE MUEVE LA CAJA
Pf=[0.5;-0.4;0];

Pi=Pc;
Pd=Pf-Pi;
%----------------------------------------
%FUNCION CON LA TRAYECTORIA
[qt1,qt2,qt1g,qt2g,qt111,qt222,qt1f,qt2f]=desplaza(Pd,qf1(:,1),qf2(:,1),robot,delta_t,coordposicion,coord);
%-----------------------------------------
%GUARDAMOS TODOS LOS RECORRIDOS PARA REPRESENTARLOS
qtotal1=[qt11 qt1 qt1g qt111 qt1f];
qtotal2=[qt12 qt2 qt2g qt222 qt2f];
graficapath(qtotal1,qtotal2,robot)%REPRESENTA LOS RECORRIDOS DE AMBOS ROBOTS
%
%SE PUEDE LLAMAR AQUI A LA FUNCION REPRESENTA, CON TODAS LAS
%COORDENADAS,PERO PARA TESTEAR SE HA DEJADO DESPUES DE CADA CALCULO DE CADA
%CONJUNTO DE POSICION ARTICULAR.
%-----------------------------------------------------
%PUESTO QUE NO ESTAMOS GUARDANDO EL TIEMPO, AL USAR SEL MISMO DELTA_T EN
%TODA LA SIMULACION PODEMOS CREAR EL VECTOR AHORA
time=0:delta_t:(size(qtotal1,2)-1)*delta_t;
%SE REPRESENTAN LAS COORDENADAS ARTICULARES EN CADA PUNTO
figure, xlabel('Tiempo'), ylabel('q1(red) q2(blue)'), title('Coordenadas articulares'), hold on
plot(time,qtotal1,'r');
plot(time,qtotal2,'b');
%---------------------------------------------------------

