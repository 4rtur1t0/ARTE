    %Partimos del conocimiento de las posiciones articulares (q), velocidad del
    %extremo(v0), ángulo de salida (teta0).
q=[q1 q2 q3];
v0=V0(I);
A01=dh(q(1),0,1,0);
A12=dh(q(2),0,1,0);
A23=dh(q(3),0,1,0);
A03=A01*A12*A23;
xt=A03(1,4);
yt=A03(2,4);
drawrobot3d(robot,q);%dibujo el robot en dicha posición articular en un sistema de referencia de 3 dimensiones.
xf=xt+xr;
x=0:xf/49:xf;
y0=(x*tan(teta0*pi/180))-(4.905*(x.^2/((v0^2)*(cos(teta0*pi/180))^2)))+yt;
y=[];
for i=1:1:50
    y(i)=y0(51-i);
end
plot(x,y,'r--');%Represento la gráfica de la función del tiro parabólico.
xlim([-1 10])
ylim([-2 7])
hold on

plot([-0.2,0.2],[3.05,3.05],'b') %dibuja el aro de la canasta.
plot([-9,9],[0,0],'g') %dibuja el suelo.
plot([-0.2,-0.2],[4.1,0],'b') %dibuja el palo de la canasta.

plot([0,0.5],[3.05,0.5*tan(teta*pi/180)+3.05],'g'); % Representamos el ángulo mínimo de entrada.

Vx=-v0*cos(teta0*pi/180);
Vy=v0*sin(teta0*pi/180);
modulo = norm([Vx,Vy]); % vamos a escalar todas las velocidades con el mismo factor 'modulo', para que las velocidades no queden demasiado grandes al dibujarlas.

% A continuacion vamos a calcular el politopo de velocidades en la
% configuracion (q1,q2,q3) actual.
J = compute_jacobian(robot,[q1,q2,q3]);
J = J(1:2,:);
% disp(J);
wmax = 5;
N = 20;
dw = 2*wmax/(N-1);
Vels = []; % genera el politopo de velocidades barriendo las 3 velocidades articulares entre los limites minimo y maximo.
for w1=-wmax:dw:wmax
    for w2=-wmax:dw:wmax
        for w3=-wmax:dw:wmax
            V = J*[w1;w2;w3];
            V = V/modulo; % Escalamos todos los vectores de velocidad igual que la velocidad de disparo, para representarlos con la misma escala.
            Vels = [Vels;V'];
        end
    end
end
x1=X(I);
y1=Y(I);
plot3(x1+Vels(:,1),y1+Vels(:,2),Vels(:,2)*0 - 0.01,'c.'); % Dibuja el politopo de velocidades, centrado en el efector final.
% (La coordenada z de estos puntos, la pinto en -0.01 para asegurarme de
% que el politopo se representa por debajo del resto de graficos).

quiver(x(50),y(50),Vx/modulo,Vy/modulo,0,'MaxHeadSize',1);
