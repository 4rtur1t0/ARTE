% Mecanismo 3RPR
% 4º Grado Ingeniería Electrónica y Automática Industrial
% Asignatura: Robótica
% 2013/2014

% Merlos Ortega, Juan Antonio
% Pérez Sotobal, Enrique




function q = inversekinematic_3RPR(robot, T)

fprintf('\nCalculando la cinematica inversa para el robot %s', robot.name);

u=T(1:3,1);
phi = atan2(u(2), u(1));

h=robot.h;

%Posiciones de los puntos A,B y C.
xA=T(1,4);
yA=T(2,4);

xB=xA+h*cos(phi);
yB=yA+h*sin(phi);

xC=xA+h*cos(phi+pi/3);
yC=yA+h*sin(phi+pi/3);


%Matriz de transformación al sistema de coordenadas del segundo brazo.
T2=robot.robot2.T0;

%el mismo punto expresado en dicho sistema
P=inv(T2)*[xB; yB; 0; 1];
%lo introducimos en T2
T2(1:3,4)=P(1:3);


%Matriz de transformación al sistema de coordenadas del tercer brazo.
T3=robot.robot3.T0;
%el mismo punto expresado en dicho sistema
P=inv(T3)*[xC; yC; 0; 1];
%lo introducimos en T3
T3(1:3,4)=P(1:3);

%Resolvemos la cinematica inversa en cada brazo
q1=inversekinematic_2dofplanarRP(robot.robot1, T);

q2=inversekinematic_2dofplanarRP(robot.robot2, T2);

q3=inversekinematic_2dofplanarRP(robot.robot3, T3);


   
q =[q1; q2; q3];
 



