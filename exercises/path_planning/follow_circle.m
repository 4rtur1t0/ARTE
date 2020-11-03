function follow_circle()

global configuration
% Cerramos todo
close all
% angulo de avance sobre el círculo (rige el número de puntos que se
% generan sobre la trayectooria)
delta_th = 0.1;


% Carga cualquier robot de la librer�a
% robot = load_robot('MITSUBISHI','PA-10');
robot = load_robot('ABB','IRB140');

T=eye(4);

% Generamos los 3 puntos aleatorios cerca de un entorno predefinido
% EJERCICIO: se recomienda al alumno que los cambie estos parámetros para entender su
% funcionamiento
p1=[0.4 0.4 0]+0.2*rand(1,3);
p2=[0 0.4 0.4]+0.2*rand(1,3);
p3=[-0.2 0.4 0.2]+0.2*rand(1,3);

plot3(p1(:,1),p1(:,2),p1(:,3),'ro');
hold on;
plot3(p2(:,1),p2(:,2),p2(:,3),'go');
plot3(p3(:,1),p3(:,2),p3(:,3),'bo');

% Llamamos a la funci�n "circlefit3d". Devuelve el centro y radio del
% círculo que contiene los puntos p1 p2 y p3
[center,rad,v1,v2] = circlefit3d(p1,p2,p3);

% Generamos los vectores normalizados
vp1 = p1(:)-center(:);
vp1 = vp1/norm(vp1);
vp2 = p2(:)-center(:);
vp2 = vp2/norm(vp2);
vp3 = p3(:)-center(:);
vp3 = vp3/norm(vp3); 

% Dibujamos los vectores generados
draw_my_vector(vp1*rad,center,'vp1')
draw_my_vector(vp2*rad,center,'vp2')
draw_my_vector(vp3*rad,center,'vp3')
draw_my_vector(v1,center,'v1')
draw_my_vector(v2,center,'v2')

% �ngulos "theta" con respecto a "v1"
th1 = find_theta(v1, v2, vp1);
th2 = find_theta(v1, v2, vp2);
th3 = find_theta(v1, v2, vp3);

% Vamos de "th1" a "th2" y despu�s de "th2" a "th3"
a1 = th1:0.1:th2;
a2 = th2:0.1:th3;
b = [a1 a2];

puntos=zeros(length(b),3);
for i=1:length(b)
    a = b(i);
    p = center(:) + rad*v1(:)*cos(a) + rad*v2(:)*sin(a);
    %plot3(p(1),p(2),p(3),'r.');
    puntos(i,:)=p;
end
axis equal;grid on;rotate3d on;

figure,
hold on, grid, plot3(puntos(:,1),puntos(:,2),puntos(:,3)), title('Trajectory in space'), xlabel('X (m)'), ylabel('Y (m)')

qs=[];
for i=1:size(puntos,1)
    T(1:3,4)=puntos(i,:)';    
    qinv = inversekinematic(robot, T);
    %select the joint coordinates in qinv which are closest to the 
    %current joint position robot.q
    q=select_closest_joint_coordinates(qinv, robot.q);
    qs=[qs q];
    robot.q=q;%update robot.q here
end

drawrobot3d(robot, qs(:,1))
adjust_view(robot)

% get the reference of the robot figure
h=figure(configuration.figure.robot);

%Andrea: te pongo el código para pintar la trayectoria articular
for i=1:size(qs,2)
    q = qs(:,i);
    drawrobot3d(robot, q);
    plot3(puntos(:,1),puntos(:,2),puntos(:,3)), title('Trajectory in space'), xlabel('X (m)'), ylabel('Y (m)')
    pause(0.01)
end

hold on, grid, plot3(puntos(:,1),puntos(:,2),puntos(:,3)), title('Trajectory in space'), xlabel('X (m)'), ylabel('Y (m)')

figure, hold, plot(qs(1,:), 'r'),plot(qs(2,:), 'g'), plot(qs(3,:), 'b'), plot(qs(4,:), 'c'), 
plot(qs(5,:), 'm.'), plot(qs(6,:), 'y.'),
legend('q_1 (rad)','q_2 (rad)','q_3 (rad)', 'q_4 (rad)', 'q_5 (rad)', 'q_6 (rad)' ), title('Joint trajectories'), xlabel('Step number')

end

% Definimos la funci�n "draw_my_vector"
function draw_my_vector(V,p0,text_label)
p1=p0(:)+V(:);
vect_arrow(p0,p1,'k',2)
text(p1(1)+0.005, p1(2)+0.005, p1(3)+0.005, text_label, 'FontWeight', 'bold','HorizontalAlignment', 'Center', 'FontSize', 15);
end

% Definimos la funci�n "find_theta"
function theta = find_theta(v1, v2, vp)
cth = dot(v1, vp);
sth = dot(v2, vp);
theta = atan2(sth, cth);
end
