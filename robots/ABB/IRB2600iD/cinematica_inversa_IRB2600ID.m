
function q=cinematica_inversa_IRB2600ID(robot,T)

%inicializo las 8 posibles soluciones en un vector de tamaño 6
q=zeros(6,8);

A06=directkinematic(robot,q);
teta=eval(robot.DH.theta);
d=eval(robot.DH.d);
a=eval(robot.DH.a);
alfa=eval(robot.DH.alpha);

% Pm: para obtener la posición de la muñeca le restamos a la posición del
% extremo, dada por la ultima columna de la matriz T obtenida por
% cinemática directa la longitud del eslabón 6 en la dirección de z6
Pm = T(1:4,4) - d(6)*T(1:4,3);

%obtenemos q1 como la arcotangente de la posición de la muñeca en y, y en x
%2 soluciones de q1= q1 y q1+pi
q1=atan2(Pm(2), Pm(1));
q1_prima=q1+pi;

%primera solucion de q2
[q2_1(1), q2_1(2)]=solve_q2(q1,Pm);
%segunda solucion de q2
[q2_2(1), q2_2(2)]=solve_q2(q1_prima,Pm);

%primera solucion de q3
[q3_1(1), q3_1(2)]=solve_q3(q1,Pm);
%segunda solucion de q3
[q3_2(1), q3_2(2)]=solve_q3(q1_prima,Pm);

%Se duplica cada columna para introducir posteriormente las diferentes
%soluciones para theta4, theta5 y theta6.
q = [q1         q1         q1        q1       q1_prima  q1_prima   q1_prima   q1_prima;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0];
%Me quedo solo con la parte real de las soluciones
q=real(q);

%Normalizo q entre [-pi, pi]
q1_prima=normalize(q1_prima);
q2_1(1)=normalize(q2_1(1));
q2_1(2)=normalize(q2_1(2));
q2_2(2)=normalize(q2_2(2));
q2_2(1)=normalize(q2_2(1));
q3_1(1)=normalize(q3_1(1));
q3_1(2)=normalize(q3_1(2));
q3_2(2)=normalize(q3_2(2));
q3_2(1)=normalize(q3_2(1));


q(:,1)=solve_ultimas_3(q1, q2_1(1), q3_1(1),-1,T);
q(:,2)=solve_ultimas_3(q1, q2_1(1), q3_1(1),1,T);
q(:,3)=solve_ultimas_3(q1, q2_1(2), q3_1(2),1,T);
q(:,4)=solve_ultimas_3(q1, q2_1(2), q3_1(2),-1,T);
q(:,5)=solve_ultimas_3(q1_prima, q2_2(1), q3_2(1),1,T);
q(:,6)=solve_ultimas_3(q1_prima, q2_2(1), q3_2(1),-1,T);
q(:,7)=solve_ultimas_3(q1_prima, q2_2(2), q3_2(2),1,T);
q(:,8)=solve_ultimas_3(q1_prima, q2_2(2), q3_2(2),-1,T);




function [q2_1,q2_2]=solve_q2(q1,Pm)

d    = [0.445  0      0     0.795     0    0.085];
a    = [0.15      0.9   0.115    0       0    0];
alfa = [-pi/2  0    -pi/2  pi/2  -pi/2   0];
A01 = dh(q1, d(1), a(1), alfa(1));
L2=a(2);
L3=d(4);

A2=a(3);
l3 = sqrt (A2^2 + L3^2);
%posición de la muñeca en el sistema de referencia 1
Pm_1=inv(A01)*Pm;
beta=atan2(-Pm_1(2),Pm_1(1));
R=sqrt((Pm_1(1))^2+(Pm_1(2))^2);
gamma = (acos((L2^2+R^2-l3^2)/(2*R*L2)));
if ~isreal(gamma)
    disp('WARNING:inversekinematic_irb1600id: the point is not reachable for this configuration, imaginary solutions'); 
    gamma = real(gamma);
end
%codo_arriba
q2_1=-gamma-beta+pi/2;

%codo_abajo
q2_2=+gamma-beta+pi/2;



function [q3_1 q3_2]=solve_q3(q1,Pm)

d    = [0.445  0      0     0.795     0    0.085];
a    = [0.15      0.9   0.115    0       0    0];
alfa = [-pi/2  0    -pi/2  pi/2  -pi/2   0];
A01 = dh(q1, d(1), a(1), alfa(1));
L2=a(2);
L3=d(4);

A2= a(3);
l3 = sqrt(A2^2 + L3^2);
phi=acos((A2^2+l3^2-L3^2)/(2*A2*l3));

%posición de la muñeca en el sistema de referencia 1
Pm_1=inv(A01)*Pm;
R=sqrt((Pm_1(1))^2+(Pm_1(2))^2);
eta = (acos((L2^2 + l3^2 - R^2)/(2*L2*l3)));
if ~isreal(eta)
   disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
   %eta = real(eta);
end
eta_2=-eta;
q3_1 = pi - phi - eta;
q3_2 = pi - phi + eta;


%Resuleve la muñeca esferica, siendo M=1 para muñeca arriba y M=-1 para
%muñeca abajo

function q=solve_ultimas_3(q1,q2,q3,M,T)

d    = [0.445  0      0     0.795     0    0.085];
a    = [0.15      0.9   0.115    0       0    0];
alfa = [-pi/2  0    -pi/2  pi/2  -pi/2   0];
% 	Cálculo de la matriz de transformación A03
A01 = dh(q1, d(1), a(1), alfa(1));
A12 = dh(q2-pi/2, d(2), a(2), alfa(2));
A23 = dh(q3, d(3), a(3), alfa(3));
A03 = A01 * A12 * A23;



x3 = A03(1:3,1);
y3 = A03(1:3,2);
z3 = A03(1:3,3);
z4 =cross(z3,T(1:3,3));	% Vector orientación a: T(1:3,3)
z4=z4/norm(z4);

if norm(z4) <= 0.000001
    if M == 1 %Muñeca arriba
        q4=0;
    else
        q4=-pi; %Muñeca abajo
    end
end;



sq4 = M*dot(z4,x3);
cq4 = M*dot(z4,-y3);
q4  = atan2(sq4,cq4);

% Solución de la quinta articulación: q5
z5  = T(1:3,3);		% Vector de orientación a: T(1:3,3)

A34 = dh(q4, d(4), a(4), alfa(4));
A04 = A03 * A34;
x4  = A04(1:3,1);
y4  = A04(1:3,2);

sq5 = dot(z5,-x4);	% Vector de orientación a: T(1:3,3)
cq5 = dot(z5,y4);	% Vector de orientación a: T(1:3,3)
q5  = atan2(sq5,cq5);

% Solución de la sexta articulación: q6
y6  = T(1:3,2);	% Vector de orientación s: T(1:3,2)

A45 = dh(q5, d(5), a(5), alfa(5));
A05 = A04 * A45;
y5  = A05(1:3,2);
x5  = A05(1:3,1);
cq6 = dot(T(1:3,1),-x5);	% Vector de orientación n: T(1:3,1)
sq6 = dot(T(1:3,1),-y5);	% Vector de orientación s: T(1:3,2)
q6  = atan2(sq6,cq6);

% Vector de coordenadas articulares
q  = [q1 q2 q3 q4 q5 q6]';





