function [xxx,yyy,zzz]=letras
load hershey
global palabra;
%palabra='AM I A ROBOT?';
sz=size(palabra);
%figure
%hold on
xprev=0;
xx=[];
yy=[];

espacio=0;
%figure
%yprev=0;
% accede a cada letra B
for i= 1:sz(2)
    if palabra(i)==' '
       espacio=0.4;
         continue
    end
B = hershey{palabra(i)};


%Se muestran por pantalla las coordenadas y se marca con NaN cada uno de
%los segmentos de la letra. Son los trazos que pueden dibujarse sin
%levantar el lapiz de la superficie
B.stroke;
%X=[(B.stroke(1,:)+xprev+0.2) B.stroke(2,:)].*3; %Esfera
%x= [2*X(1)/(1+X(1)^2+X(2)^2), 2*X(2)/(1+X(1)^2+X(2)^2), (-1+X(1)^2+X(2)^2)/(2+X(1)^2+X(2)^2)];

%obten coordenadas "unitarias" x, y
x=B.stroke(1,:)+xprev+0.2+espacio;
y=B.stroke(2,:);
xx=[xx x];
yy=[yy y];

%plotea mostrando las coordenadas de los puntos en coordenadas unitarias
%%figure
%hold on
%plot(x,y,'*')
xprev=max(xx);
yprev=y(length(y));
xx=[xx NaN];
yy=[yy yprev];
espacio=0;
end
xx=xx-0.2;
% Con las lineas siguientes obtenemos una trayectoria que permite levantar
% el lapiz cuando se finaliza cada trazo
path = [[xx;yy]; zeros(1,size([xx;yy],2))]; 
k = find(isnan(path(1,:)));
path(:,k) = path(:,k-1); 
path(3,k) = 0.2;
path=path/max(path(2,:));
% % plotea el camino
xxx=path(1,:);
yyy=path(2,:);
zzz=path(3,:);
%plot3(path(1,:),path(2,:), path(3,:))
end