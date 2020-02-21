%Coordenadas tomadas por el "radar" del misil objetivo
fprintf('Inserte posición 1 del misil objetivo\n');
% x0=input('x0: ');
% y0=input('y0: ');
% z0=input('z0: ');
 x0=500;y0=5000;z0=4000;   
fprintf('Inserte posición 2 del misil objetivo\n');
% x1=input('x1: ');
% y1=input('y1: ');
% z1=input('z1: ');
x1=1000;y1=4500;z1=4050;

%Cálculo de velocidades del misil objetivo, estableciendo en t1=1 s el
%intervalo entre las coordenadas tomadas
fprintf('Calculando velocidades del misil objetivo...\n');
t1=1; %Intervalo entre las coordenadas tomadas

vmx=x1-x0/t1;
vmy=y1-y0/t1;
vmz=z1-z0/t1;
fprintf('Vmx= %4.5f m/s\nVmy= %4.5f m/s\nVmz= %4.5f m/s\n',abs(vmx),abs(vmy),abs(vmz));

t2=150; %Intervalo de tiempo desde que se tomar la última posición del misil objetivo hasta que se produce la colisión en s.
v=4000; %Velocidad del antimisil en m/s.

fprintf('Calculando coordenadas de impacto respecto al misil...\n');
xcm=x1+vmx*t2;
ycm=y1+vmy*t2;
zcm=z1+vmz*t2;

fprintf('Xcm= %4.10f m\nYcm= %4.10f m\nZcm= %4.10f m\n',xcm,ycm,zcm);

%Sistema de ecuaciones de Xc e Yc para hallar Beta y t3
beta=atan2(ycm,xcm);
t3=xcm/(v*cos(beta));

%Calculo de tiempo en posicionarse


fprintf('Tiempo que tarda el antimisil en llegar al punto de colisión=%4.5f s\nTiempo que tarda el misil en llegar al punto de colisión=%4.5f s\n',t3,t2);

%Habiendo calculado Beta y t3 pasamos a calcular Gamma
L1=2.100017631;
L2=7.5;
gamma=asin((zcm+0.5*9.8*(t3)^2-L1)/(v*t3));

fprintf('Beta=%4.5f rad(%4.5fº)\nGamma=%4.5f rad(%4.5fº)\n',beta,beta*180/pi,gamma,gamma*180/pi);
q=[beta;gamma];


fprintf('Comprobando calculos....\n');
fprintf('Calculando coordenadas de impacto respecto al antimisil...\n');
xca=v*cos(beta)*t3;     xca=real(xca);
yca=v*sin(beta)*t3;     yca=real(yca);
zca=v*sin(gamma)*t3-0.5*9.8*(t3)^2+L1;      zca=real(zca);
fprintf('Xca= %4.10f m\nYca= %4.10f m\nZca= %4.10f m\n',xca,yca,zca);

%Las coordenadas deben ser las mismas ya sean calculadas respecto al misil
%o respecto al antimisil.
 if xca==xcm && yca==ycm && zca==zcm
     fprintf('Coordenadas de impacto calculadas correctamente!\n');
 else    fprintf('Coordenadas de impacto incorrectas!\n');
 end;

%Giros máximos de las articulaciones q1(Beta) y q2(Gamma)
if beta<=2*pi && gamma<=pi/2 fprintf('El disparo es viable\n');
else fprintf('El disparo es inviable\n');
end;

%El intervalo tiempo(para alcanzar el punto de colisión) para el antimisil debe ser menor que el del misil, ya que sino debería ser lanzado
%en un tiempo negativo, lo cual es imposible.
if t3<t2 fprintf('Misil objetivo alcanzable\n');
else fprintf('Misil objetivo inalcanzable\n');
end;