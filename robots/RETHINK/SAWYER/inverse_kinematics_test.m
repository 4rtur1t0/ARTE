
 %T = [-1 0 0 -0.2; 
  %   0   1  0  0.3;
   %%0  0  0   1];
T = [1 0 0 0.3; 
    0   1  0  0.3;
    0 0 1    0.5;
    0  0  0   1];

q = [0 0 0 0 0 0 0]';
qinv = inverse_kinematics_sawyer(robot, T, q)

T_reach = directkinematic(robot, qinv)

'diff T-Treach'
T-T_reach