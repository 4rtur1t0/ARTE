% Mecanismo 3RPR
% 4º Grado Ingeniería Electrónica y Automática Industrial
% Asignatura: Robótica
% 2013/2014

% Merlos Ortega, Juan Antonio
% Pérez Sotobal, Enrique


function robot = parameters()

robot.name='2DOF planar arm';

robot.DH.theta='[q(1) 0]';
robot.DH.d='[0  0]';
robot.DH.a='[1  q(2)]';
robot.DH.alpha='[0  0]';

robot.DOF = 2;

robot.kind=['R' 'T'];

robot.J=[];




robot.inversekinematic_fn = 'inversekinematic_2dofplanarRP(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';

robot.maxangle =[deg2rad(-180) deg2rad(180); %Eje 1, min, max
                0 2]; %Eje 2, min, max, metros
                

robot.velmax = []; 

robot.accelmax=robot.velmax/0.1; % 0.1 aquí es un tiempo de aceleración

% Velocidad máxima del efector final
robot.linear_velmax = 0; %m/s


%Sistema de referencia de la base
robot.T0 = eye(4);

%INICIALIZACIÓN DE VARIABLES REQUERIDAS PARA LA SIMULACIÓN
%posición, velocidad y aceleración
robot=init_sim_variables(robot);
robot.path = pwd;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GRÁFICOS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.graphical.has_graphics=1;
robot.graphical.color = [25 20 40];
robot.graphical.draw_transparent=0;
robot.graphical.draw_axes=1;
robot.graphical.axes_scale=1;
robot.axis=[-2.2 2.2 -2.2 2.2 0 2.2]
robot = read_graphics(robot);


%INICIALIZACIÓN DE VARIABLES REQUERIDAS PARA LA SIMULACIÓN
%posición, velocidad y aceleración
robot.q=[0 0]';
robot.qd=[0 0]';
robot.qdd=[0 0]';
robot.time = [];

robot.q_vector=[];
robot.qd_vector=[];
robot.qdd_vector=[];


robot.last_target=directkinematic(robot, robot.q);
robot.last_zone_data = 'fine';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GRÁFICOS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.graphical.has_graphics=1;
robot.graphical.color = [25 20 40];
robot.graphical.draw_transparent=0;
robot.graphical.draw_axes=1;
robot.graphical.axes_scale=1;
robot.axis=[-2.2 2.2 -2.2 2.2 0 2.2]
robot = read_graphics(robot);


%INICIALIZACIÓN DE VARIABLES REQUERIDAS PARA LA SIMULACIÓN
%posición, velocidad y aceleración
robot.q=[0 0]';
robot.qd=[0 0]';
robot.qdd=[0 0]';
robot.time = [];

robot.q_vector=[0 0];
robot.qd_vector=[0 0];
robot.qdd_vector=[0 0];


robot.last_target=directkinematic(robot, robot.q);
robot.last_zone_data = 'fine';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARÁMETROS DINÁMICOS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

robot.dynamics.friction=0;

%Masa de los brazos (kg)
robot.dynamics.masses=[1 1];
robot.dynamics.r_com=[-0.5      0         0; 
                     -0.5      0         0];

m1 = robot.dynamics.masses(1);
m2 = robot.dynamics.masses(2);

L1 = robot.DH.a(1);
L2 = robot.DH.a(2);


%Matrices de inercia de cada eslabón

robot.dynamics.Inertia=[0   0   m1*L1/3    0	0	0;
                        0   0   m2*L2/3    0	0	0];


%Inercia del rotor
robot.motors.Inertia=[0 0];

robot.motors.G=[1  1];


%Factor de fricción viscosa del motor
robot.motors.Viscous = [0  0];
%Fricción de Coulomb del motor
%Tc+, Tc-
robot.motors.Coulomb = [0	0;
            0	0];
%Constantes del motor       
%                       R(Ohm)  L(H)     Kv (V/rad/s):cte de vel.  Kp (Nm/A):cte de par    Max_current (A) 
robot.motors.constants=[0.345  0.273e-3       2.3474e-05               84.9e-3                 139];        %estas corresponden a Maxon, 167132;
  

        
        

