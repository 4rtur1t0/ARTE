fprintf('Inserte posición 1 del misil objetivo\n');
% x0=input('x0: ');
% y0=input('y0: ');
% z0=input('z0: ');
  x0=10000;y0=13000;z0=80000;   
fprintf('Inserte posición 2 del misil objetivo\n');
% x1=input('x1: ');
% y1=input('y1: ');
% z1=input('z1: ');
x1=7500;y1=9800;z1=79990;

fprintf('Calculando velocidades del misil objetivo...\n');
t1=1; %Intervalo de toma de muestras
vmx=abs((x1-x0)/t1); sx=abs(x1-x0)/(x1-x0);
vmy=abs((y1-y0)/t1); sy=abs(y1-y0)/(y1-y0);
vmz=abs((z1-z0)/t1); sz=abs(z1-z0)/(z1-z0);
fprintf('vmx= %4.5f m/s\nvmy= %4.5f m/s\nvmz= %4.5f m/s\n',vmx,vmy,vmz);

t2=100; %Intervalo desde que llegan los datos hasta que se produce la colisión
v=5000; %Velocidad del antimisiles
xcm=vmx*t2*sx+x1;
ycm=vmy*t2*sy+y1;
zcm=vmz*t2*sz+z1;

beta=atan2(ycm,xcm);
t3=(xcm/(v*cos(beta)));
fprintf('t3= %4.5f s\n',t3);
gamma=asin((0.5*9.8*(t3)^2+vmz*t2)/(v*t3));


fprintf('Calculando coordenadas de la colisión...\n');
xca=v*cos(beta)*t3;
yca=v*sin(beta)*t3;
%zca=v*sin(gamma)*t3-0.5*9.8*(t3)^2+2.100017631;
fprintf('Colisión en:\nXc=%4.5f m\nYc=%4.5f m\nZc=%4.5f m\n',xca,yca,zcm);
%fprintf('\n\nComprobación:\nXc:%4.5f=%4.5f\nYc:%4.5f=%4.5f\nZc:%4.5f=%4.5f\n\n',xcm,xca,ycm,yca,zcm,zca);

mx=xca;
my=yca;
mz=zcm;
fprintf('\ngamma= %4.5f\ngamma= %4.5f\n',gamma,atan2(mz,mx));
       
       fprintf('Calculando velocidades y tiempo en alcanzar el objetivo...\n');
       vox=v*cos(beta);
       t=mx/vox;
       voy=v*sin(beta);
       voz=v*sin(gamma);
       fprintf('Vox=%4.5f m/s\nVoy=%4.5f m/s\nVoz=%4.5f m/s\n',vox,voy,voz);
       fprintf('Tiempo=%4.5f s\n',t);
       

       