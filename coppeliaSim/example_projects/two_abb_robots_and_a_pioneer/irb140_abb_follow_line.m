function irb140_abb_follow_line()
    % Aqui lo que hacemos es inicializar la comunicacion ARTE con COPPELIA
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    coppelia.n_collision_objects = 0;
    coppelia.robots{1}.n_joints = 6;
    coppelia.robots{1}.end_effector.n_joints = 2;
    coppelia.robots{2}.n_joints = 6;
    coppelia.robots{2}.end_effector.n_joints = 2;
    coppelia.dt = 50/1000; % default 50 ms
    coppelia = coppelia_start(coppelia); 
    
    %handles de los sensores utilizados en coppelia    
    [retorno, sensor] = coppelia.sim.simxGetObjectHandle(coppelia.clientID, 'sensor_fin', coppelia.sim.simx_opmode_blocking);
    [ret_vision, sensor_vision] = coppelia.sim.simxGetObjectHandle(coppelia.clientID, 'Vision_sensor', coppelia.sim.simx_opmode_blocking);
    [ret_vision0, sensor_vision0] = coppelia.sim.simxGetObjectHandle(coppelia.clientID, 'Vision_sensor0', coppelia.sim.simx_opmode_blocking);
    [ret_motor_D, motorD] = coppelia.sim.simxGetObjectHandle(coppelia.clientID, 'Pioneer_p3dx_leftMotor', coppelia.sim.simx_opmode_blocking);
    [ret_motor_I, motorI] = coppelia.sim.simxGetObjectHandle(coppelia.clientID, 'Pioneer_p3dx_rightMotor', coppelia.sim.simx_opmode_blocking);
    
    %inicializamos las velocidades del robot movil
    %tambien podemos poner como velocidad 0 del robot movil en parametros
    %de coppelia
    [returnCodeMd]=coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID,motorD,0,coppelia.sim.simx_opmode_blocking)
    [returnCodeMi]=coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID,motorI,0,coppelia.sim.simx_opmode_blocking)
    
    % cargamos los robot IRB en arte
    global robot
    robot = load_robot('ABB', 'IRB140');
    global robot2
    robot2 = load_robot('ABB', 'IRB140');
    
    primeraVez=1;
    %cogemos los 3 cubos del  suelo y los ponemos en la cinta
    for i=0:2 
        while 1
             %cogemos los 3 cubos del  suelo y los ponemos en la cinta
             if primeraVez
                suelo_a_cinta_R1(coppelia,i,robot);
                primeraVez=0;
             end
            %aqui leemos el detector final de la cinta par poner en marcha el
            %segundo robot.
            [returnCode, detectado]=coppelia.sim.simxReadProximitySensor(coppelia.clientID,sensor,coppelia.sim.simx_opmode_blocking);
            coppelia.sim.simxSynchronousTrigger(coppelia.clientID);
            
            %cogemos los 3 cubos de la cinta y los ponemos en el robot movil
            if detectado
                cinta_a_movil_R2(coppelia,i,robot);
                primeraVez = 1;
                break
            end
        end
    end
    %Cuando tenemos todos loc cubos en el robot movil iniciamos movimiento  
    mover_pioneer (coppelia, sensor_vision, sensor_vision0 , motorD, motorI);
    for i=0:2 
        while 1
            %cogemos los 3 cubos del robot movil y los ponemos en la cinta
             if primeraVez
                movil_a_cinta_R1(coppelia,i,robot);
                primeraVez=0;
             end
            %aqui leemos el detector final de la cinta par poner en marcha el
            %segundo robot.
            
            [returnCode, detectado]=coppelia.sim.simxReadProximitySensor(coppelia.clientID,sensor,coppelia.sim.simx_opmode_blocking);
            coppelia.sim.simxSynchronousTrigger(coppelia.clientID);
            
            %cogemos los 3 cubos de la cinta y los ponemos la mesa
            if detectado
                cinta_a_mesa_R2(coppelia,i,robot);
                primeraVez = 1;
                break
            end
        end
    end
    pause(5);
    coppelia_stop(coppelia);
    
   
end
    
function suelo_a_cinta_R1(coppelia,i,robot)

% Posicion para coger el cubo
T2 = [-1 0 0 0.54;
      0 1 0  -0.025+i*0.15;
      0 0 -1 0.24;
      0 0 0 1];
% Posicion en cinta para dejar cubo
T3 = [-1 0 0 0.5;
       0 1 0  -0.52;
       0 0 -1 0.50; 
       0 0 0 1];
%Deja cubo en la cinta   
T32 = [-1 0 0 0.5;
       0 1 0  -0.52;
       0 0 -1 0.42;
       0 0 0 1];
%Vuelve a Posicion reposo
T4 = [-1 0 0 0.54;
       0 1 0  0.0;
       0 0 -1 0.5;
       0 0 0 1];
open_gripper(coppelia, 1);


if i==0
    [qt, qdt]=ComputeAbsJPath(robot, [-0.0002 0.3161 0.0344 0.0 1.2202 -0.0002], [0 0 0 0 0 0], 30);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);
else
    robot.q = [-0.0002 0.3161 0.0344 0.0 1.2202 -0.0002]';
    robot.qd=[0.2845 0.2703 -0.3944 -0.0 0.1241 0.2845]';
end

[qt, qdt] = ComputeLPath(robot, T2, 22);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

close_gripper(coppelia, 1);
[qt, qdt]=ComputeLPath(robot, T3, 20);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

[qt, qdt]=ComputeLPath(robot, T32, 20);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

open_gripper(coppelia, 1);
[qt, qdt] = ComputeLPath(robot, T4, 30);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

end

function cinta_a_movil_R2(coppelia,i,robot2)
%Se posiciona para coger cubo de cinta
T1 = [-1 0 0 0.38;
       0 1 0  0.2;
       0 0 -1 0.425;
       0 0 0 1];
%Coge el cubo de la cinta
T2 = [-1 0 0 0.66;
       0 1 0  0.33;
       0 0 -1 0.405;
       0 0 0 1];
%Se eleva un poco una vez cogido
T22 = [-1 0 0 0.66;  
       0 1 0  0.33;
       0 0 -1 0.455;
       0 0 0 1];
%Vuelve a posicion espera
T5 = [-1 0 0 0.25; 
       0 1 0  -0.3;
       0 0 -1 0.6;
       0 0 0 1];
%Se posiciona encima de robot movil para dejar cubo   
T6 = [-1 0 0 -0.1+i*0.065; 
       0 1 0  -0.6;
       0 0 -1 0.7;
       0 0 0 1];
%Deja cubo en robot movil
T7 = [-1 0 0 -0.1+i*0.065; 
       0 1 0  -0.6;
       0 0 -1 0.495;
       0 0 0 1];

%Movimiento que hace para inicializar robot a una posicion
open_gripper(coppelia, 2);
[qt, qdt]=ComputeAbsJPath(robot2, [0.1 0.2 0.3 0.1 0.2 0.3], [0 0 0 0 0 0], 30);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T1, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T2, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);
close_gripper(coppelia, 2);

[qt, qdt] = ComputeLPath(robot2, T22, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T6, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T7, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);
open_gripper(coppelia,2);


[qt, qdt] = ComputeLPath(robot2, T5, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);
close_gripper(coppelia, 2);

end

function movil_a_cinta_R1(coppelia,i,robot)
%Initial point
T11 = [-1 0 0 0.1786;
       0 1 0  0.4223;
       0 0 -1 0.6;
       0 0 0 1];
   
T12 = [-1 0 0 0.2478;
       0 1 0  0.4296;
       0 0 -1 0.6;
       0 0 0 1];
   
T13 = [-1 0 0 0.317;
       0 1 0  0.4296;
       0 0 -1 0.6;
       0 0 0 1];

% Posicion para coger el cubo
T21 = [-1 0 0 0.2786;
      0 1 0  0.4223;
      0 0 -1 0.475;
      0 0 0 1];
 
T22 = [-1 0 0 0.32;
       0 1 0  0.4296;
       0 0 -1 0.475;
       0 0 0 1];
   
T23 = [-1 0 0 0.38;
       0 1 0  0.4296;
       0 0 -1 0.475;
       0 0 0 1];

   
% Final point arriba
T3 = [-1 0 0 0.5;
       0 1 0  -0.52;
       0 0 -1 0.45;
       0 0 0 1];
   
T32 = [-1 0 0 0.5;
       0 1 0  -0.52;
       0 0 -1 0.42;
       0 0 0 1];
%Posicion reposo
T4 = [-1 0 0 0.54;
       0 1 0  0.0;
       0 0 -1 0.5;
       0 0 0 1];
open_gripper(coppelia, 1);

if i==0
    [qt, qdt]=ComputeAbsJPath(robot, [-0.0002 0.3161 0.0344 0.0 1.2202 -0.0002], [0 0 0 0 0 0], 30);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);
else
    robot.q = [-0.0002 0.3161 0.0344 0.0 1.2202 -0.0002]';
    robot.qd=[0.2845 0.2703 -0.3944 -0.0 0.1241 0.2845]';
end

if i == 0
    T1 = T11;
    T2 = T21;
elseif i == 1
    T1 = T12;
    T2 = T22;
elseif i == 2
    T1 = T13;
    T2 = T23;
end

[qt, qdt] = ComputeLPath(robot, T1, 20);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot, T2, 20);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

close_gripper(coppelia, 1);
[qt, qdt] = ComputeLPath(robot, T1, 20);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

[qt, qdt]=ComputeLPath(robot, T3, 20);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

[qt, qdt]=ComputeLPath(robot, T32, 20);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

open_gripper(coppelia, 1);
robot.q = qt(:,end);
robot.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot, T4, 50);
move_robot(coppelia, 1, qt, qdt)
robot.q = qt(:,end);
robot.qd=qdt(:,end);

end

function cinta_a_mesa_R2(coppelia,i,robot2)

T1 = [-1 0 0 0.38;
       0 1 0  0.2;
       0 0 -1 0.425;
       0 0 0 1];
% Punto coger en cinta
T2 = [-1 0 0 0.66;
       0 1 0  0.33;
       0 0 -1 0.405;
       0 0 0 1];
T22 = [-1 0 0 0.66;  %T2 con Z mas alta
       0 1 0  0.33;
       0 0 -1 0.455;
       0 0 0 1];
%Posicion reposo
T5 = [-1 0 0 0.25; %Posicion intermedia
       0 1 0  0.3;
       0 0 -1 0.6;
       0 0 0 1];
   
T6 = [-1 0 0 -0.1+i*0.065; %Pinza elevada antes de dejar la pieza
       0 1 0  0.6;
       0 0 -1 0.7;
       0 0 0 1];

T7 = [-1 0 0 -0.1+i*0.065; %Posicion final
       0 1 0  0.6;
       0 0 -1 0.55;
       0 0 0 1];

open_gripper(coppelia, 2);
[qt, qdt]=ComputeAbsJPath(robot2, [0.1 0.2 0.3 0.1 0.2 0.3], [0 0 0 0 0 0], 30);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T1, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T2, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);
close_gripper(coppelia, 2);

[qt, qdt] = ComputeLPath(robot2, T22, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = MoveLPath(robot2, T5, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T6, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T7, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);
open_gripper(coppelia,2);

[qt, qdt] = MoveLPath(robot2, T6, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);

[qt, qdt] = ComputeLPath(robot2, T5, 20);
move_robot(coppelia, 2, qt, qdt)
robot2.q = qt(:,end);
robot2.qd=qdt(:,end);


end

function [fin_movil]= mover_pioneer (coppelia, sensor_vision, sensor_vision0, motorD, motorI)

while 1
    k= 0.18;
    vLeft=2.9;%3;
    vRight=2.9;%3;
 coppelia.sim.simxSynchronousTrigger(coppelia.clientID);
 [returnCode2, det_vision, otro]=coppelia.sim.simxGetVisionSensorImage2(coppelia.clientID,sensor_vision,0,coppelia.sim.simx_opmode_blocking);
 [returnCode0, det_vision0, otro0]=coppelia.sim.simxGetVisionSensorImage2(coppelia.clientID,sensor_vision0,0,coppelia.sim.simx_opmode_blocking);  

 if otro(:,:,1)<50 && otro0(:,:, 1)<50  %los 2 ven negro
     vLeft=vLeft;
     vRight=vRight;
 elseif otro(end,end,1)>150 && otro0(end,end,1)>150 %llega a punto final
     fin_movil=1;
     %mando antes de salir el valor de velocidad = 0, si no lo mandamos al
     %salir se pone de nuevo en marcha el robot movil
     [returnCodeMd]=coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID,motorD,0,coppelia.sim.simx_opmode_blocking)
    [returnCodeMi]=coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID,motorI,0,coppelia.sim.simx_opmode_blocking)
     break;
 elseif otro(:,:,1)<50 && otro0(:,:,1)>200 %solo veo derecha
      vLeft=vLeft*-k;
      vRight=vRight*k;
 elseif otro0(:,:,1)<50 && otro(:,:,1)>200 %solo veo irquierda
      vLeft=vLeft*k;
      vRight=vRight*-k;
 end
 %mandamos velocidad a las ruedas del robot_movil
[returnCodeMd]=coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID,motorD,vRight,coppelia.sim.simx_opmode_blocking)
[returnCodeMi]=coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID,motorI,vLeft,coppelia.sim.simx_opmode_blocking)
end

end

