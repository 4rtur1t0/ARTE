
function robot = parameters()

robot.name= 'GP7';


robot.path = 'robots/MOTOMAN/GP7';

robot.DH.theta =    '[q(1)      q(2)+pi/2   q(3)    q(4)    q(5)    q(6)]';
robot.DH.d =        '[0.330     0           0       -0.440  0       -0.080]';
robot.DH.a =        '[0.040     0.445       0.040   0       0       0]';
robot.DH.alpha =    '[pi/2      0           -pi/2   pi/2    -pi/2   0]';

robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_GP7(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';



robot.DOF = 6;


robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];


robot.maxangle =[   deg2rad(-170) deg2rad(170);
                    deg2rad(-65) deg2rad(150);
                    deg2rad(-113) deg2rad(255);
                    deg2rad(-190) deg2rad(190);
                    deg2rad(-135) deg2rad(135);
                    deg2rad(-360) deg2rad(360)];


robot.velmax = [deg2rad(375);  
                deg2rad(315);  
                deg2rad(410);  
                deg2rad(550);  
                deg2rad(550); 
                deg2rad(1000)];

robot.linear_velmax = 3.0; 
robot.accelmax = robot.velmax / 0.1;



robot.T0 = eye(4); 




robot=init_sim_variables(robot);
robot.path = pwd;



robot.graphical.has_graphics=1;
robot.graphical.color = [23 30 243]./255;

robot.graphical.draw_transparent=0;

robot.graphical.draw_axes=1;

robot.graphical.axes_scale=1;

robot.axis=[-0.5 1 -0.5 1 0 1];

robot = read_graphics(robot);



%PUESTO QUE ES UN ROBOT MUY SIMILAR AL GP8, MANTENGO LAS MISMAS DINAMICAS
%NO ESTA TESTEADO

robot.has_dynamics=1;


robot.dynamics.friction=0;


mass_factor = 32 / ((2464.42 + 5353.42 + 2608.24 + 2087.33 + 400.90 + 16.07));
robot.dynamics.masses=[(2464.42 * mass_factor)  (5353.42 * mass_factor) (2608.24 * mass_factor) (2087.33 * mass_factor) (400.90 * mass_factor) (16.07 * mass_factor)];


size_factor = 1 / 1000; 
                        
robot.dynamics.r_com=[  (-29.98 * size_factor)      (-46.50 * size_factor)      (0.45 * size_factor);   %(rx, ry, rz) link 1
                        (-162.72 * size_factor)     (29.19 * size_factor)       (2.44 * size_factor);   %(rx, ry, rz) link 2
                        (19.87 * size_factor)       (-0.98 * size_factor)       (22.33 * size_factor);  %(rx, ry, rz) link 3
                        (-0.22 * size_factor)       (148.34 * size_factor)      (1.16 * size_factor);   %(rx, ry, rz) link 4
                        (-0.11 * size_factor)       (-0.02 * size_factor)       (-17.22 * size_factor); %(rx, ry, rz) link 5
                        (0.00 * size_factor)        (-0.22 * size_factor)       (4.66 * size_factor)];  %(rx, ry, rz) link 6


intertia_factor = 1 / (1000  * 1000 * 1000);  
robot.dynamics.Inertia=[(13782869.81 * intertia_factor) (9789239.23 * intertia_factor)      (18374891.67 * intertia_factor)     (4806926.23 * intertia_factor)      (-13736.36 * intertia_factor)	(-7425.15 * intertia_factor);
                        (43933685.46 * intertia_factor) (231969173.67 * intertia_factor)    (208909704.41 * intertia_factor)    (-26305468.52 * intertia_factor)	(26254.69 * intertia_factor)    (-827615.83 * intertia_factor);
                        (9165010.01 * intertia_factor)  (11499392.36 * intertia_factor)     (8521027.15 * intertia_factor)      (-1185.38 * intertia_factor)    	(36312.29 * intertia_factor)	(605655.44 * intertia_factor);
                        (58934726.03 * intertia_factor) (3442622.78 * intertia_factor)	    (58396681.09 * intertia_factor)     (-71578.98 * intertia_factor)	    (420309.01 * intertia_factor)	(-1131.89 * intertia_factor);
                        (582557.40 * intertia_factor)   (598347.64 * intertia_factor)	    (258584.89 * intertia_factor)	    (-284.56 * intertia_factor)	        (50.96 * intertia_factor)	    (1971.39 * intertia_factor);
                        (3896.87 * intertia_factor)     (3828.25 * intertia_factor)	        (6717.18 * intertia_factor)	        (-0.01 * intertia_factor)	        (-8.78 * intertia_factor)	    (0.00 * intertia_factor)];


robot.motors=load_motors([3 2 2 2 1 1]);

robot.motors.G=[10 15 5 4 4 4];

