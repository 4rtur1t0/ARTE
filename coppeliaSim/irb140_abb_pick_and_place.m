% EJEMPLO SENCILLO DE INTERFAZ ENTRE ARTE Y COPPELIA.

function irb140_abb_pick_and_place()
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    coppelia.n_collision_objects = 0;
    coppelia.robot.name = 'R1';
    coppelia.robot.n_joints = 6;
    coppelia.robot.end_effector.n_joints = 2;
    
    coppelia = coppelia_start(coppelia);
    % escribid una funcion que este entre coppelia start y coppelia_stop
    pick_and_place(coppelia)
  
    coppelia_stop(coppelia);
end


function pick_and_place(coppelia)
%load in arte
global robot
robot = load_robot('ABB', 'IRB140');

    %T1, pre pick point
    T1 = [0  1  0 0.52;
         1  0  0  -0.1;
          0  0 -1 0.45;
          0  0  0 1];
    % pick point
    T2 = [0  1 0 0.52;
          1 0 0  -0.1;
         0 0 -1 0.33;
         0 0 0 1];   
    % Release point
    T3 = [-1 0 0 0.1;
         0 1 0  -0.5;
         0 0 -1 0.6;
          0 0 0 1]; 
    T4 = [-1 0 0 0.1;
         0 1 0  -0.5;
        0 0 -1 0.5;
        0 0 0 1]; 
    % Podeis ir a una posicion q arbitraria
    q = [0.1 0.1 0.1 0.8 0.8 0.8]';
    set_joint_target_trajectory(coppelia, q)

    %podeis ir a una posicion que sea la cinematica inversa deseada
    qinv = inversekinematic(robot, T1);
    set_joint_target_trajectory(coppelia, qinv(:,1))
    % Esto abre la pinza
    open_gripper(coppelia); 
    % podéis esparar un múltiplo del dt de Coppelia (step size de simulación de
    % coppelia, tip. 50 ms. Esperar a que se abra la pinza
    coppelia_wait(coppelia, 10)
    %podeis ir a una posicion que sea la cinematica inversa deseada
    qinv = inversekinematic(robot, T2);
    set_joint_target_trajectory(coppelia, qinv(:,1))
 
    % cerrar pinza
    close_gripper(coppelia);
    % Esperar a que se cierre
    coppelia_wait(coppelia, 1)
    % subir
    qinv = inversekinematic(robot, T1);
    set_joint_target_trajectory(coppelia, qinv(:,1))

    qinv = inversekinematic(robot, T3);
    set_joint_target_trajectory(coppelia, qinv(:,1))

    qinv = inversekinematic(robot, T4);
    set_joint_target_trajectory(coppelia, qinv(:,1))

    % Esto abre la pinza
    open_gripper(coppelia); 
    coppelia_wait(coppelia, 1)

    % VOLVEMOS A la posicion inicial
    set_joint_target_trajectory(coppelia, q)
end







