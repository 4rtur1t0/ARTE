function Representacion_en_esfera(y)
%Representacion en esfera
%X=[1 1];
%x=[2*X(1)/(1+X(1)^2+X(2)^2), 2*X(2)/(1+X(1)^2+X(2)^2), (-1+X(1)^2+X(2)^2)/(1+X(1)^2+X(2)^2)];
global radio;
%Esfera de radio deseado en coordenadas esféricas
theta = linspace(0,2*pi,50);
phi = linspace(-pi,pi,50);
r = ones(1,50)*radio;
origen=y(1:3,4);
%Construcción de malla
[phi,theta] = meshgrid(phi,theta);
r = meshgrid(r);

%La conversión se realiza después de construir la malla
[X,Y,Z] = sph2cart(theta,phi,r);
X=X+origen(1);
Y=Y+origen(2);
Z=Z+origen(3);
%plot3(x(1),x(2),x(3),'*')
hold on
surf(X,Y,Z), axis equal
colormap((gray));
end
