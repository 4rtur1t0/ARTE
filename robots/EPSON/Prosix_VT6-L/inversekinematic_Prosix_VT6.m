function q = inversekinematic_Prosix_VT6(robot, T)

q=zeros(6,8);

d = eval(robot.DH.d);

L6=d(6);
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

Z6 = T(1:3,3);

%Cálculo de Pm
Pm = [Px Py Pz]' - L6*Z6;

%Cálculo de q1
q1=atan2(Pm(2), Pm(1));

%Llamamos a las funciones de cálculo de q2 y q3
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);

q2_2=solve_for_theta2(robot, [q1+pi 0 0 0 0 0 0], Pm);

q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);

q3_2=solve_for_theta3(robot, [q1+pi 0 0 0 0 0 0], Pm);


%Tenemos 8 posibles soluciones
% q1 y q1+pi.
%Para cada una de ellas, tenemos 2 posibles soluciones de q2 y q3, codo
%arriba y codo abajo.
%A continuación se incluye una matri con las 8 posibles soluciones de la
%cinemática inversa.

q = [q1         q1         q1        q1       q1+pi   q1+pi   q1+pi   q1+pi;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0];

%Solo la parte real de la solución
q=real(q);

%Normalizamos q a [-pi, pi] debido a rangos no simétricos
q(1,:) = normalize(q(1,:));
q(2,:) = normalize(q(2,:));

% Resolución de las 3 últimas articulaciones con método algebráico
for i=1:2:size(q,2),
    
    qtemp = solve_spherical_wrist2(robot, q(:,i), T, 1,'geometric'); %Muñeca arriba
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i)=qtemp;
    
    qtemp = solve_spherical_wrist2(robot, q(:,i), T, -1, 'geometric'); %Muñeca abajo
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i+1)=qtemp;
end

%Cálculo de q2 codo arriba y codo abajo
function q2 = solve_for_theta2(robot, q, Pm)

d = eval(robot.DH.d);
a = eval(robot.DH.a);

L2=a(2);
L3=d(4);

T01=dh(robot, q, 1);

p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(-p1(2), p1(1));
gamma = (acos((L2^2+r^2-L3^2)/(2*r*L2)));

if ~isreal(gamma)
    disp('WARNING:inversekinematic_prosixt6L: el punto no es alcanzable para esta solución, soluciones imaginarias'); 
end

q2(1) = pi/2 - beta - gamma; %codo arriba
q2(2) = pi/2 - beta + gamma; %codo abajo

%Cálculo de q3 codo arriba y codo abajo
function q3 = solve_for_theta3(robot, q, Pm)

d = eval(robot.DH.d);
a = eval(robot.DH.a);

L2=a(2);
L3=d(4);

T01=dh(robot, q, 1);

p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

eta = (acos((L2^2 + L3^2 - r^2)/(2*L2*L3)));

if ~isreal(eta)
   disp('WARNING:inversekinematic_prosixvt6L: el punto no es alcanzable para esta solución, soluciones imaginarias'); 
end

q3(1) = pi/2 - eta; %codo arriba
q3(2) = eta - 3*pi/2;%codo abajo



