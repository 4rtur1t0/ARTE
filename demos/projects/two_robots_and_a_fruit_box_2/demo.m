function demo()

%-------SISTEMA AMBIDEXTRO DE MANIPULACIÓN ROBÓTICA-------

%--------------------------AUTORES------------------------
%Agulló Soto, Rubén -> 
%Bonmatí Campello, Raúl -> 
%Sánchez Martí, Joaquín -> 
%---------------------------------------------------------

%-----------------------PROCEDIMIENTO---------------------
%Paso 1: Cargar el robot UR10 -> robot1 = load_robot
%Paso 2: Volver a cargar el robot UR10 con otro nombre -> robot2 = load_robot
%Paso 3: Cargar la caja (fruit_box) -> caja = load_robot
%Paso 4: Añadirle un giro de pi en la primera articulacion -> robot2.DH.theta='[q(1)+pi q(2)+pi/2 q(3) q(4)-pi/2 q(5) q(6)]'
%Paso 5: Desplazar el segundo robot hasta la posicion "-1" en el eje "Y" -> robot2.T0(2,4)=-1
%Paso 6: Comentar la line 42 de "drawrobot3D" y poner "h=0";
%Opcional:(para que se vea mejor)
    %1.- Cambiar el color del robot 1 -> robot1.graphical.color=[0 0 255]./255;
    %2.- Cambiar el color del robot 2 -> robot2.graphical.color=[0 0 255]./255;
    %3.- Quitar los ejes del robot 1 -> robot1.graphical.draw_axes=0;
    %4.- Quitar los ejes del robot 2 -> robot2.graphical.draw_axes=0;
    %5.- Quitar los ejes de la caja -> caja.graphical.draw_axes=0;
%Paso 7: Llamar a la funcion -> demo(robot1,robot2,caja)
%---------------------------------------------------------

robot1 = load_robot('UR', 'UR10')
robot2 = load_robot('UR', 'UR10')

caja = load_robot('equipment/objects', 'fruit_box')
robot2.DH.theta='[q(1)+pi q(2)+pi/2 q(3) q(4)-pi/2 q(5) q(6)]'
robot2.T0(2,4)=-1
robot1.graphical.color=[0 0 255]./255;
robot2.graphical.color=[0 0 255]./255;
robot1.graphical.draw_axes=0;
robot2.graphical.draw_axes=0;
caja.graphical.draw_axes=0;

t = 1;
[pi]=angulos_euler(robot1,[-0.132 -0.967 -1.289 0.279 -0.021 0]);
[pi2]=angulos_euler(robot1,[0.1440 0.9641 1.3023 -0.0121 0.0544 -1.0145]);
[pf]=angulos_euler(robot1,[-0.119 0.405 -1.626 -0.74 0 2.011]);
[pf2]=angulos_euler(robot1,[0.1498 -0.4106 1.6997 0.8706 0.0273 -2.0933]);

xpunto1=generacion_trayectorias(pi,pf);
q1=jacobiana(robot1,xpunto1);

xpunto2=generacion_trayectorias(pi2,pf2);
q2=-(jacobiana(robot2,xpunto2));

%-----Descomentar para elegir punto de vista--------------------------
%drawrobot3d(robot1,q1(t,:));
%drawrobot3d(robot2,q2(t,:),1);
%drawrobot3d(caja,zeros(1,6),1); 
%pause
%---------------------------------------------------------------------
while t<99
    drawrobot3d(robot1,q1(t,:)); % dibuja el primer robot
    %hold on
    drawrobot3d(robot2,q2(t,:),1); % dibuja el segundo robot
    %----------dibuja la caja----------
    T=directkinematic(robot1,q1(t,:));
    caja.T0=T;
    caja.T0(1:3,1:3)=[1 0 0;0 0 -1;0 1 0];
    caja.T0(3,4)=caja.T0(3,4)-0.05;
    caja.T0(2,4)=caja.T0(2,4)-0.1;
    drawrobot3d(caja,zeros(1,6),1); 
    %----------------------------------
    pause(0.05);
   %hold off
    t=t+1;
end
end