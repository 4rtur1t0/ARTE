%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS devuelve una estructura que contiene todos los datos de   %
%   nuestro robot                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = parameters()

robot.name= 'IRB2600ID';

%Directorio donde se contienen todo los archivos del robot
robot.path = 'robots/abb/IRB2600ID';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.445 0 0 0.795 0 0.085]';
robot.DH.a='[0.15 0.9 0.115 0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';

robot.J=[];


robot.inversekinematic_fn = 'cinematica_inversa_IRB2600ID(robot,T)';

%Numero de grados de libertad
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%Ángulos de rotación mínimos y máximos por cada eje (en rad)
robot.maxangle =[deg2rad(-180) deg2rad(180);    %Eje 1
                deg2rad(-95) deg2rad(155);      %Eje 2
                deg2rad(-180) deg2rad(75);      %Eje 3
                deg2rad(-175) deg2rad(175);     %Eje 4: 
                deg2rad(-120) deg2rad(120);     %Eje 5
                deg2rad(-400) deg2rad(400)];    %Eje 6: 

%Velocidad máxima absoluta por cada eje
robot.velmax = [deg2rad(175);   %Eje 1, rad/s
                deg2rad(175);   %Eje 2, rad/s
                deg2rad(175);   %Eje 3, rad/s
                deg2rad(360);   %Eje 4, rad/s
                deg2rad(360);   %Eje 5, rad/s
                deg2rad(500)];  %Eje 6, rad/s

robot.accelmax=robot.velmax/0.1;    %Aceleración máxima (0.1 es un tiempo de aceleración)
robot.linear_velmax = 1.0;          %Velocidad máxima del efector final

%base reference system 
robot.T0 = eye(4);

%Inicialización de las variables de posición, velocidad y aceleración
robot=init_sim_variables(robot);

% GRAFICAS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 20 20]./255;
%Transparencia
robot.graphical.draw_transparent=0;
%Dibujar los sistemas DH
robot.graphical.draw_axes=0;
%Escala de los sistemas DH
robot.graphical.axes_scale=1;
%Vista por defecto del robot
robot.axis=[-3 3 -3 3 0 3.5];
%Lectura de los gráficos
robot = read_graphics(robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot.has_dynamics=1;

%Masa de cada eslabón (kg)
robot.dynamics.masses=[ 44.48 28.31 80.88 66.72 4.46 0.6];

%Centro de masas de cada eslabón respecto su propio sistema de referencia
                    %  (rx      ry      rz) de cada eje
robot.dynamics.r_com=[ 0        0.172    0;     
                      -0.45     0       -0.247;
                      -0.0985   0        0;
                       0       -0.3213   0;
                       0        0       -0.01;
                       0        0       -0.01];

%Matriz de inercia de cada eslabón respecto el sistema de referencia DH
                        % Ixx       Iyy       Izz    Ixy	Iyz	   Ixz
robot.dynamics.Inertia=[10.7855    1.2741    9.1625   0      0      0;
                         1.1608    6.8549    5.7926   0      0      0;
                         1.0901    6.5883    6.5268   0      0      0;
                         6.1849    0.0774    6.2399   0      0      0;
                         0.0140    0.0140    0.0105   0      0      0;
                         0.0008    0.0008    0.0014   0      0      0];

%Selección de motores
robot.motors=load_motors([6 6 7 8 1 1]);
%Reductora para cada eje
robot.motors.G=[100 100 100 100 100 100];
