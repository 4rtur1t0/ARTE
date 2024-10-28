% move last three joints
robot = load_robot('ABB', 'IRB140');
robot.graphical.draw_transparent = 1;
pause
% Wrist singularity
q = [pi/8, pi/8, pi/8, pi/8, 0, pi/8];
drawrobot3d(robot, q)
J = manipulator_jacobian(robot, q)
det(J) % columnas 4 y 6 iguales

% Singularities at the edge of the workspace
q = [0, pi/2, -pi/2, pi/4, pi/4, pi/4];
drawrobot3d(robot, q)
J = manipulator_jacobian(robot, q)
det(J) % combinación lineal: 
%-0.0325*(J(5,:)+ J(6,:)) == J(1,:)

% Singularities at Z0
T = [1 0 0 0;
     0 1 0 0;
     0 0 1 0.7;
     0 0 0 1];
qinv= inversekinematic(robot, T)
drawrobot3d(robot, qinv(:,1))
J = manipulator_jacobian(robot, qinv(:,1))
det(J)

% NO Singularities!
q = [pi/8, pi/8, -pi/8, -pi/8, pi/8, pi/8];
drawrobot3d(robot, q)
J = manipulator_jacobian(robot, q)
det(J) % combinación lineal: 

