function q = inversekinematic_MH5LF(robot, T)

%inicializar q
%ocho posibles soluciones son generalmente factibles
q=zeros(6,8);

%Evaluamos los parametros
A06=directkinematic(robot,q);
a = eval(robot.DH.a);
d = eval(robot.DH.d);

%Ver geometría en la referencia para este robot.
L6=d(6);

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);



%Calcula la posicion final de la muñeca siendo siendo W la coordenada Z del
%sistema de efectores finales
W = T(1:4,3);

% Pm: posicion de la muñeca
Pm = T(1:4,4) - d(6)*W;

%primera articulacion, admite dos posibles soluciones: 
% si q1 es una solucion, entonces q1+pi tambien es otra solucion
q1=atan2(Pm(2), Pm(1));
q1prima=q1+pi;

%resolvemos para q2
[q2_1(1) q2_1(2)]=solve_q2(q1,Pm,robot);
[q2_2(1) q2_2(2)]=solve_q2(q1prima,Pm,robot);


%resolvemos para q3

[q3_1(1) q3_1(2)]=solve_q3(q1,Pm,robot);
[q3_2(1) q3_2(2)]=solve_q3(q1prima,Pm,robot);


%Arrange solutions, there are 8 possible solutions so far.
% if q1 is a solution, q1* = q1 + pi is also a solution.
% For each (q1, q1*) there are two possible solutions
% for q2 and q3 (namely, elbow up and elbow up solutions)
% So far, we have 4 possible solutions. Howefer, for each triplet (theta1, theta2, theta3),
% there exist two more possible solutions for the last three joints, generally
% called wrist up and wrist down solutions. For this reason, 
%the next matrix doubles each column. For each two columns, two different
%configurations for theta4, theta5 and theta6 will be computed. These
%configurations are generally referred as wrist up and wrist down solution
q = [q1         q1         q1        q1       q1prima   q1prima   q1prima   q1prima;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0];

%nos quedamos solo con la parte real de las soluciones
q=real(q);

%normalizamos q
q=normalize(q);

%resolvemos para el resto de q's(8 soluciones posibles) donde signo =1 para
%muñeca arriba y signo=-1 muñeca abajo. Escribimos una a una cada
%combinacion posible para las 8 soluciones y las almacenamos por columnas en
%la matriz q.
q(:,1)=solve_q4q5q6(q1, q2_1(1), q3_1(1),T,-1,robot);
q(:,2)=solve_q4q5q6(q1, q2_1(1), q3_1(1),T,1,robot);
q(:,3)=solve_q4q5q6(q1, q2_1(2), q3_1(2),T,1,robot);
q(:,4)=solve_q4q5q6(q1, q2_1(2), q3_1(2),T,-1,robot);
q(:,5)=solve_q4q5q6(q1prima, q2_2(1), q3_2(1),T,1,robot);
q(:,6)=solve_q4q5q6(q1prima, q2_2(1), q3_2(1),T,-1,robot);
q(:,7)=solve_q4q5q6(q1prima, q2_2(2), q3_2(2),T,1,robot);
q(:,8)=solve_q4q5q6(q1prima, q2_2(2), q3_2(2),T,-1,robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% resolver la segunda articulacion con dos posibles soluciones, codo arriba
% y codo abajo.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q2_1 q2_2] = solve_q2(q1,Pm,robot)
%Evaluamos los parametros D-H
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%asignamos las longitudes L2 y L3 a su parametro D-H correspondiente 
L2=a(2);
L3=d(4);

A3=a(3); %Desfase
l3=sqrt(A3^2+L3^2); %Simplificar el desfase

%como conocemos q1, podemos sacar la primera matriz transformacion
T01=dh(q1, d(1), a(1), alpha(1));

%Expresamos la posicion de la muñeca en referencia al sistema 1, por
%conveniencia 
Pm_1 = inv(T01)*Pm;
R=sqrt(Pm_1(1)^2+Pm_1(2)^2);
gamma=(acos((L2^2+R^2-l3^2)/(2*L2*R)));
beta=atan2(Pm_1(2),Pm_1(1));

if ~isreal(gamma)
    disp('WARNING:inversekinematic_MH5LF: the point is not reachable for this configuration, imaginary solutions'); 
    gamma = real(gamma);
end

%nos devuelve dos posibles soluciones, codo arriba y codo abajo
%el orden aqui es importante y esta coordinado con la con la funcion

q2_1=-(pi/2)+beta+gamma; %codo arriba
q2_2=-(pi/2)+beta-gamma; %codo abajo


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% resolver la tercera articulacion con dos posibles soluciones, codo arriba
% y codo abajo.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q3_1 q3_2] = solve_q3(q1,Pm,robot)
%evaluamos los parametros D-H
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%asignamos las longitudes L2 y L3 a su parametro D-H correspondiente 
L2=a(2);
L3=d(4);

A3= a(3); %desfase
l3 = sqrt(A3^2 + L3^2); %simplificamos el desfase

%the angle phi is fixed
phi=atan2(L3,A3);

%como conocemos q1, podemos sacar la primera matriz transformacion
T01=dh(q1, d(1), a(1), alpha(1));


%expresamos la posicion de la muñeca en referencia al sistema 1
Pm_1 = inv(T01)*Pm;
R=sqrt(Pm_1(1)^2+Pm_1(2)^2);
eta = (acos((L2^2 + l3^2 - R^2)/(2*L2*l3)));

if ~isreal(eta)
   disp('WARNING:inversekinematic_fanuc_mate: the point is not reachable for this configuration, imaginary solutions'); 
   eta = real(eta);
end

%devuelve dos posibles soluciones, codo arrib y codo abajo
q3_1 = phi-(pi-eta); %codo arriba
q3_2 = phi-(pi+eta); %codo abajo


%elimina las soluciones complejas de q
function  qreal = arrange_solutions(q)
qreal=q(:,1:4);

%sum along rows if any angle is complex, for any possible solutions, then v(i) is complex
v = sum(q, 1);

for i=5:8,
    if isreal(v(i))
        qreal=[qreal q(:,i)]; %store the real solutions
    end
end

qreal = real(qreal);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Resolvemos la muñeca esferica para sus tres ultimas posiciones articulares
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q=solve_q4q5q6(q1,q2,q3,T,signo,robot)

%evaluamos los parametros
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%calculamos la matriz de transformacion A03
A01=dh(q1,d(1),a(1),alpha(1));
A12=dh(q2+pi/2,d(2),a(2),alpha(2));
A23=dh(q3,d(3),a(3),alpha(3));

A03=A01*A12*A23;

x3= A03(1:3,1);
y3= A03(1:3,2);
z3= A03(1:3,3);


x6=T(1:3,1);
y6=T(1:3,2);
z6=T(1:3,3);

z5=z6;


z4=cross(z3,z6); %producto escalar
z4=z4/norm(z4); %sacamos el modulo

if norm(z4)<= 0.00000001
    if signo==1;    %muñeca arriba
        q4=0;
    else
        q4=-pi;  %muñeca abajo
    end
end

%calculamos q4
cosq4=signo*dot(z4,y3);
senq4=signo*dot(z4,-x3);

q4=atan2(senq4,cosq4);

%calculamos q5
A34=dh(q4,d(4),a(4),alpha(4));
A04=A03*A34;

x4=A04(1:3,1);
y4=A04(1:3,2);
 
cosq5=dot(z5,-y4);
senq5=dot(z5,x4);

q5=atan2(senq5,cosq5);


%calculamos q6
A45=dh(q5,d(5),a(5),alpha(5));
A05=A04*A45;

x5=A05(1:3,1);
y5=A05(1:3,2);

cosq6=dot(T(1:3,1),y5);
senq6=dot(T(1:3,1),-x5);

q6=atan2(senq6,cosq6);


%vector de las articulaciones
q=[q1 q2 q3 q4 q5 q6];

q=normalize(q);
