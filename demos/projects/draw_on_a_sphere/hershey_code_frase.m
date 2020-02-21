load hershey
palabra='D';
sz=size(palabra);
figure
%hold on
xprev=0;
%yprev=0;
% accede a cada letra B
for i= 1:sz(2)
B = hershey{palabra(i)};


%Se muestran por pantalla las coordenadas y se marca con NaN cada uno de
%los segmentos de la letra. Son los trazos que pueden dibujarse sin
%levantar el lapiz de la superficie
B.stroke;
%X=[(B.stroke(1,:)+xprev+0.2) B.stroke(2,:)].*3; %Esfera
%x= [2*X(1)/(1+X(1)^2+X(2)^2), 2*X(2)/(1+X(1)^2+X(2)^2), (-1+X(1)^2+X(2)^2)/(2+X(1)^2+X(2)^2)];

%obten coordenadas "unitarias" x, y
x=B.stroke(1,:)+xprev+0.2;
y=B.stroke(2,:);

%plotea mostrando las coordenadas de los puntos en coordenadas unitarias
%%figure
hold on
%plot(x,y,'*')
xprev=max(x);

end
% Con las lineas siguientes obtenemos una trayectoria que permite levantar
% el lapiz cuando se finaliza cada trazo
path = [ 0.25*B.stroke; zeros(1,size(B.stroke,2))]; 
k = find(isnan(path(1,:)));
path(:,k) = path(:,k-1); 
path(3,k) = 0.2;

% % plotea el camino
plot3(path(1,:),path(2,:), path(3,:),'*')