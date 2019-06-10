function robot = parameters()

robot.name='EPSON_PROSIX_C4L';

robot.path = 'robots/EPSON/C4L_techo';

robot.DH.theta='[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.166 0 0 0.4 0 0.065]';
robot.DH.a='[0.1 0.4 0 0 0 0]';
robot.DH.alpha='[-pi/2 0 -pi/2 pi/2 -pi/2 0]';

%Sitema de referencia de la base
robot.T0 = eye(4);
%Giramos el sistema para colocarlo en el techo
Rotacion=[-1 0 0;0 1 0;0 0 -1];
robot.T0=[Rotacion,zeros(3,1);0,0,0,1];

%Al llamar a la cinemática inversa, le pasamos la T respecto de la inversa
%de T0
robot.inversekinematic_fn = 'inversekinematic_prosixc4l(robot,inv(robot.T0)*T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';

robot.DOF=6;

%rotacional: 0, translacional: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%Ángulo de rotación min y max en rad/s
robot.maxangle =[deg2rad(-170) deg2rad(170); %1
                deg2rad(-65) deg2rad(160); %2
                deg2rad(-200) deg2rad(43); %3
                deg2rad(-200) deg2rad(200); %4
                deg2rad(-135) deg2rad(135); %5
                deg2rad(-360) deg2rad(360)]; %6

%Velocidad máxima absoluta de cada articulación
robot.velmax = [deg2rad(275); % 1, rad/s
                deg2rad(275); % 2, rad/s
                deg2rad(289); % 3, rad/s
                deg2rad(555); % 4, rad/s
                deg2rad(555); % 5, rad/s
                deg2rad(720)];% 6, rad/s
    
robot.accelmax=robot.velmax/0.1; % 0.1 es el tiempo de aceleración
            
% Velocidad máxima del efector final
robot.linear_velmax = 2.5; %m/s







%Inicialización de variables
%Posición, velocidad y aceleración
robot=init_sim_variables(robot);
robot.path = pwd;


%Gráficos
robot.graphical.has_graphics=1;
robot.graphical.color = [255 102 51]./255;
%Transparencia
robot.graphical.draw_transparent=0;
%Dibujar sistema D-H
robot.graphical.draw_axes=1;
%Tamaño sistema D-H
robot.graphical.axes_scale=1;
%Ejes
robot.axis=[-0.85 0.45 -0.65 0.65 -1 0.20];
%Leer grádicos stl
robot = read_graphics(robot);



% Parámetros dinámicos

robot.has_dynamics=1;

%Fricción
robot.dynamics.friction=0;

%Masas de los eslabones (kg)
robot.dynamics.masses=[10.7 3.79 2.159 2.655 0.5528 0.1208];

%Distancia del centro de gravedad al centro del sistema D-H
robot.dynamics.r_com=[-0.083      0.036           0; %(rx, ry, rz) link 1
                     -0.3304	 0	         0; %(rx, ry, rz) link 2
                     0	         0	         0; %(rx, ry, rz) link 3
                     0       -0.179           0; %(rx, ry, rz) link 4
                     0       0                0;%(rx, ry, rz) link 5
                     0       0           0.072];%(rx, ry, rz) link 6

%Matrices de inercia de cada eslabón respecto al sistema D-H
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, en cada línea
robot.dynamics.Inertia=[0.0829      0.1637	0.1566   	0.032	0	0;
    0.0050     .4643	.4185	0	0	0;
    .0056	.0045	.0056	0	0	0;
    0.1119	0.0027	0.1117	0	0	0;
    0.00063	0.00042	0.00063	0	0	0;
    0.6652e-3	0.6652e-3	0.0570e-3	0	0	0];


%Selección de motores
robot.motors=load_motors([2 3 2 1 1 1]);
%Selección de reductoras
robot.motors.G=[250 300 250 250 250 200];