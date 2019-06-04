%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   
%   ABB IRB2600. 
%   Author: Jose Albarranch Martinez. 
%   date:   20/12/2018
%
%   CONTIENE LOS PARAMETROS CINEMATICOS Y DINAMICOS QUE CONFIGURAN
%   EL ROBOT ABB IRB2600 v-20/1.65
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = parameters()

robot.name= 'ABB_IRB2600';

%robot.path = 'robots/abb/IRB2600';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.445 0 0 0.795 0 0.085]';
robot.DH.a='[0.150 0.700 0.115 0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';

robot.J=[];

robot.inversekinematic_fn = 'inversekinematic_irb2600(robot, T)';

%Numero de Grados de Libertad
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%Angulos de rotacion maximos y minimos.
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                deg2rad(-95) deg2rad(155); %Axis 2, minimum, maximum
                deg2rad(-180) deg2rad(75); %Axis 3
                deg2rad(-400) deg2rad(400); %Axis 4: ILIMITADO
                deg2rad(-120) deg2rad(120); %Axis 5
                deg2rad(-400) deg2rad(400)]; %Axis 6: ILIMITADO

%Velocidad maxima en cada articulacion en rad/s.
robot.velmax = [deg2rad(175); %Axis 1, rad/s
                deg2rad(175); %Axis 2, rad/s
                deg2rad(175); %Axis 3, rad/s
                deg2rad(360); %Axis 4, rad/s
                deg2rad(360); %Axis 5, rad/s
                deg2rad(500)];%Axis 6, rad/s

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            % end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet

%base reference system 
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=1;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-2 2 -2 2 0 2.5];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
%link 0 m=88.389kg
robot.dynamics.masses=[ 0 66.263 33.449 65.225 16.675 1.026 0.135];


%COM of each link with respect to own reference system
robot.dynamics.r_com=[  0.043	 0.007	 0.389; %(rx, ry, rz) link 1
                        0.000	-0.321	-0.173;%(rx, ry, rz) link 2
                        0.044	 0.014	 0.075; %(rx, ry, rz) link 3
                        0.000	 0.000	 0.604; %(rx, ry, rz) link 4
                        0.000	 0.000	 0.000; %(rx, ry, rz) link 5
                        0.000	 0.000	 0.072];%(rx, ry, rz) link 6
                    
                    

% %COM of each link with respect to initial reference system
% robot.dynamics.r_com=[  0.043       0.0069    0.3896; %(rx, ry, rz) link 1
%                         0.150       -0.765    0.172;%(rx, ry, rz) link 2
%                         1.220       0.193	  0.0139	  ; %(rx, ry, rz) link 3
%                         1.260       0         0.750; %(rx, ry, rz) link 4
%                         1.260       0.945     0.000     ; %(rx, ry, rz) link 5
%                         1.260       1.017       0.000     ];%(rx, ry, rz) link 6
%                     
       
                    
                    


%Inertia matrices of each link with respect to its D-H reference system.
 
%                                Ixx        Iyy     Izz     Ixy     Iyz     Ixz
   
 robot.dynamics.Inertia= [          0       0       0       0       0       0;
                               11.161    11.861     1.712    -0.046     -0.196      -1.225;
                                5.947     1.098     4.994     0.001     -1.861       0.001;
                                1.189     1.596     1.091    -0.018     -0.042      -0.270;
                                6.580     6.570     0.041    -0.000     -0.016       0.004;
                                0.001	  0.001	    0.002	  0.000	     0.000	     0.000;
                                0.001	  0.001	    0.000	  0.000	     0.000	     0.000];                       
                                       
%Los momentos de Inercia y los centros de gravedad quedan corregidos de los
%obtenidos del programa Inventor para ser ajustados al sistema de
%coordenadas designado en nuestro robot.

%robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[ 89.8 89.8 89.8 48.3 136.393 104.3 ];



