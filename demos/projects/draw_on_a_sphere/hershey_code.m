load hershey

% accede a cada letra B
B = hershey{'B'};
escala=1;
%Se muestran por pantalla las coordenadas y se marca con NaN cada uno de
%los segmentos de la letra. Son los trazos que pueden dibujarse sin
%levantar el lapiz de la superficie
B.stroke

%obten coordenadas "unitarias" x, y
x=B.stroke(1,:);
y=B.stroke(2,:);

x=x*escala;
y=y*escala;
%plotea mostrando las coordenadas de los puntos en coordenadas unitarias
figure
%plot(x, y,'*')

% Con las lineas siguientes obtenemos una trayectoria que permite levantar
% el lapiz cuando se finaliza cada trazo
path = [ escala*B.stroke; zeros(1,size(B.stroke,2))]; 
k = find(isnan(path(1,:))); %Donde sube
path(:,k) = path(:,k-1); 
path(3,k) = 0.2; %Cuanto sube

% plotea el camino
plot3(path(1,:),path(2,:), path(3,:),'*')


%%% B=
