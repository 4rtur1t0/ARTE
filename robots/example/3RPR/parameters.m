% Mecanismo 3RPR
% 4º Grado Ingeniería Electrónica y Automática Industrial
% Asignatura: Robótica
% 2013/2014

% Merlos Ortega, Juan Antonio
% Pérez Sotobal, Enrique

function robot = parameters()



%El mecanismo está dividido en tres brazos de 2 gdl
%La estructura del robot tendrá 3 brazos diferentes a los que llamaremmos:
robot1=load_robot('example','2dofplanarRP');
robot2=load_robot('example','2dofplanarRP');
robot3=load_robot('example','2dofplanarRP');


robot=[];
robot.name='3RPR parallel';
robot.robot1=robot1;
robot.robot2=robot2;
robot.robot3=robot3;

robot.nserial=3; 


robot.T0=eye(4);

robot.h=0.5; %longitud del lado del triángulo

%desplazamiento en x entre un brazo y otro
L=2.5; 

T=eye(4);
T(1,4)=L; 
robot.robot2.T0=T;

T=eye(4);
T(1,4)=L/2;
T(2,4)=L;
robot.robot3.T0=T;


robot.parallel=1;

robot.graphical=1;

robot.DOF=6; 

robot.axis=[-1.2 4.2 -1.2 3.2 0 2.2]

robot.debug=1;

%Nombres de las funciones para calcular la cinemática:
robot.inversekinematic_fn = 'inversekinematic_3RPR(robot, T)';
robot.directkinematic_fn = 'directkinematic_3RPR_numerical(robot, q)';

robot.q=zeros(1, robot.DOF);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GRÁFICOS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot.graphical.has_graphics=1;
robot.graphical.color = [25 20 40];
robot.graphical.draw_transparent=0;
robot.graphical.draw_axes=1;
robot.graphical.axes_scale=1;
robot.axis=[-2.2 2.2 -2.2 2.2 0 2.2]
end

