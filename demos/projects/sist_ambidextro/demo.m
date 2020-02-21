function demo()
%hay una trayectoria definida ya. Para otra descomentar los puntos
%iniciales y finales en "calculo_puntos" y "obtencion_q" Estas son trayectorias libres de
%puntos singulares. "Calculo_puntos2" es un script de apoyo.
disp('Seleccciona robots -> UR -> UR10 -> parameters.m')
%la seleccion del archivo es manual porque sino daba un error
robot1=load_robot('UR', 'UR10');
robot2=robot1;
robot2.T0=[1     0     0     0;0     1     0    -1;0     0     1     0;0 0     0     1];
robot2.DH.theta='[q(1)+pi q(2)+pi/2 q(3) q(4)-pi/2 q(5) q(6)]';
disp('Seleccciona robots -> equipment -> objects -> fruit_box -> parameters.m')
caja=load_robot('equipment/objects', 'fruit_box')
disp('Seleccciona robots -> equipment -> tables -> table_small -> parameters.m')
mesa=load_robot('equipment/tables', 'table_small')
mesa.T0=[1 0 0 0.3;0 1 0 -0.6;0 0 1 -0.02;0 0 0 1];

[pi,pi2,pf,pf2]=calculo_puntos(robot1);
%[pi,pf]=calculo_puntos2(robot1);
xpunto1=trayectorias(pi,pf);
xpunto2=trayectorias(pi2,pf2);
%l=1;
q1=obtencion_q(robot1,xpunto1);
q2=obtencion_q(robot2,xpunto2);
q2=-q2;
%q2=-q1
t = 1;

axis([-5,5,-5,5,-5,5]);
grid on;
disp('Press any key')
pause
while t<99

    drawrobot3d(robot1,q1(t,:)); % dibuja el primer robot
    hold on
    drawrobot3d(robot2,q2(t,:),1); % dibuja el segundo robot
    %----------------------------------
    %while l<2
    T=directkinematic(robot1,q1(t,:));
     caja.T0=T;
    caja.T0(1:3,1:3)=[1 0 0;0 0 -1;0 1 0];
    caja.T0(3,4)=caja.T0(3,4)-0.05;
     
     caja.T0(2,4)=caja.T0(2,4)-0.1;
     %l=l+1;
    %end
    %----------------------------------
    drawrobot3d(caja,zeros(1,6),1); % dibuja la caja
    drawrobot3d(mesa,zeros(1,6),1);
    pause(0.01); % Pausa, para poder ver la animación de ambos brazos
        hold off
    
    t=t+1;
end
end