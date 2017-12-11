function q = inversekinematic_IRB7600_340(robot, T)

%iniciamos q, y habrá 8 posibles soluciones, como sabemos tras estudiar la
%cinemática inversa

q=zeros(6,8);

% %Obtenemos theta
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
L6=d(6);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Obtenemos la posicion del extremo,que estará en las 3 primeras filas de la columna 3
W = T(1:3,3);

% Pm es la posición de la muñeca y se obtiene:
Pm = [Px Py Pz]' - L6*W; 

%Como sabemos,para q1 obtendremos dos valores posibles (q1) y q(1)+pi
q1=atan2(Pm(2), Pm(1));


%Resolvemos q2 con q(1)
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);
%Resolvemos q2 con q(1)+pi
q2_2=solve_for_theta2(robot, [q1+pi 0 0 0 0 0 0], Pm);

%Resolvemos q3 con q(1)
q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);
%Resolvemos q3 con q(1)+pi
q3_2=solve_for_theta3(robot, [q1+pi 0 0 0 0 0 0], Pm);


%Esta matriz nos dará los valores de q para la cinemática inversa
q = [q1         q1         q1        q1       q1+pi   q1+pi   q1+pi   q1+pi;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0];

 %Nos quedamos con la parte real de las soluciones
q=real(q);

%Note that in this robot, the joint q3 has a non-simmetrical range. In this
%case, the joint ranges from 60 deg to -219 deg, thus, the typical normalizing
%step is avoided in this angle (the next line is commented). When solving
%for the orientation, the solutions are normalized to the [-pi, pi] range
%only for the theta4, theta5 and theta6 joints.

%Normalizamos q para [-pi, pi]
q(1,:) = normalize(q(1,:));
q(2,:) = normalize(q(2,:));

% Resolvemos para las 3 ultimas articulaciones para cualquiera de las posibles combinaciones (theta1, theta2, theta3)
for i=1:2:size(q,2),
    % Usamos solve_spherical_wrist2 para la orientacion particular de cada
    % uno de los sistemas ABB. Podemos utilizar tanto el método geométrico
    % como el algebráico. La función solve_spherical_wrist2 se utiliza
    % debido a la orientacion relativa de las últimos 3 sistemas de
    % referencia de la matriz D-H
    %Aquí hemos utilizado el método algebráico, por eso mostramos el
    %geométrico como comentario.
    %qtemp = solve_spherical_wrist2(robot, q(:,i), T, 1, 'geometric'); %wrist up
    qtemp = solve_spherical_wrist2(robot, q(:,i), T, 1,'algebraic'); %wrist up
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i)=qtemp;
    
    %qtemp = solve_spherical_wrist2(robot, q(:,i), T, -1, 'geometric'); %wrist down
    qtemp = solve_spherical_wrist2(robot, q(:,i), T, -1, 'algebraic'); %wrist down
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i+1)=qtemp;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Resolvemos q2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)

%Evaluamos los parámetros
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%Mirando la geometría del robot
L2=a(2);
L3=d(4);
A2=a(3);

%Mirando la geometría del robot
%L4 es:
L4 = sqrt(A2^2 + L3^2);
%Conocida q1, obtenemos la matriz DH
T01=dh(robot, q, 1);

% Pm en coordenadas del sistema 1
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(-p1(2), p1(1));
gamma = real(acos((L2^2+r^2-L4^2)/(2*r*L2)));

%Devolvemos las dos posibles soluciones, codo arriba y codo abajo
q2(1) = pi/2 - beta - gamma; %codo arriba
q2(2) = pi/2 - beta + gamma; %codo abajo


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Resolvemos q3 para codo arriba y codo abajo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(robot, q, Pm)

%Evaluamos los parámetros
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%%Mirando la geometría del robot
L2=a(2);
L3=d(4);
A2=a(3);

%Mirando la geometría del robot
%L4 es:
L4 = sqrt(A2^2 + L3^2);

%El ángulo phi es:
phi=acos((A2^2+L4^2-L3^2)/(2*A2*L4));

%Conocido q1, obtenemos la matriz DH
T01=dh(robot, q, 1);

%Pm en coordenadas del sistema 1 es
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

eta = real(acos((L2^2 + L4^2 - r^2)/(2*L2*L4)));

%Devolvemos las dos posibles soluciones codo arriba y codo abajo
q3(1) = pi - phi- eta; 
q3(2) = pi - phi + eta; 

