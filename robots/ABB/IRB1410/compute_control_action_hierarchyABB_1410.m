
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Use a hierarchical approach to compute solution
% From Bruno Siciliano
% A General Framework for managing multiple tasks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vq] = compute_control_action_hierarchy(robot, V, q, restriction)
DOF = robot.DOF;
%init solution
N0 = eye(DOF);
x0 = zeros(DOF,1);
joint_i = 3; %this joint is considered redundant and set to 0 in the solution of the initial kinematic constraints

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initialize
x = x0;
N = N0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% restriction in V and W--> 6Dof involved
[x1, N] = kinematic_restrictionVW(robot, q, x, N, V);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%output
vq = x1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% restriction in qd3=0--> 1Dof involved only
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(restriction, 'q3_0')
  [x2, N] = q_0_restriction(robot, q, x1, N, joint_i);
  %output
  vq = x2;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% max manipulability along null space 1 DOF.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(restriction, 'max_manip')
    [x2] = max_manipulability_simple(robot, q, +1);
    %output
    vq = x2;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% min manipulability along null space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(restriction, 'min_manip')
    [x2] = max_manipulability_simple(robot, q, -1);
    %output
    vq = x2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   KINEMATIC RESTRICION--> THE end effector must follow 
% First restriction is
% J*qd = V, with V=[v w] the linear and angular speed of the end effector
% Ai*qd=bi --> J=A and V=bi
%   THIS IS ONE OF THE MAIN RESTRICTIONS AND WORKS CORRECTLY
%    Allows to compute a solution for the inverse kinematic problem
% for example.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x, N] = kinematic_restrictionVW(robot, q, x, N, V)
J = manipulator_jacobian(robot, q);
%La restricción indica Jx=V--> ecuaciones cinemáticas en Jacobiana
A=J;
b=V;
x=compute_hierarchy_sol(x, A, b, N);
N=compute_hierarchy_N(A, N);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   KINEMATIC RESTRICION V--> THE end effector must follow 
%   J*qd = V, with V=[v w] the linear and angular speed of the end effector
%   The restriction is only applied to the linear speed v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x, N] = kinematic_restrictionV(robot, q, V, x, N)
J = manipulator_jacobian(robot, q);
A=J;
%deleting the restriction in angular velocity
A(4:6,:) = zeros(3,7);%%% primer 6 si existe
b=V;
x=compute_hierarchy_sol(x, A, b, N);
N=compute_hierarchy_N(A, N);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   KINEMATIC RESTRICION W--> THE end effector must follow 
%   J*qd = V, with V=[v w] the linear and angular speed of the end effector
%   The restriction is only applied to the angular speed v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x, N] = kinematic_restrictionW(robot, q, V, x, N)
J = manipulator_jacobian(robot, q);
A=J;
%deleting the restriction in linear velocity
A(1:3,:) = zeros(3,7);
b=V;
x=compute_hierarchy_sol(x, A, b, N);
N=compute_hierarchy_N(A, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% joint i must not move 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x, N] = q_0_restriction(robot, q, x, N, joint_i)
%just restrict that the movement in joint i must be zero
A = zeros(7,7);
A(joint_i,joint_i) = 1;
b = zeros(7,1);
x=compute_hierarchy_sol(x, A, b, N);
N=compute_hierarchy_N(A, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CLOSE TO BASIC POSE
% try to increase a joint value to reach a particular base value.
% for example 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x, N] = close_to_pose_restriction2(robot, q, x, N)
%q base--> try to reach this one
qf = [0 0 0 0 0 0 0]';
A = eye(7,7);
%A(3,3) = 1;
b = 0.001*(qf-q);%zeros(7,1);
%this restriction applies a force that tries to bring q to the basic pose
%which is qf
% for i=1:length(q)
%     b(i) = 0.01*sign(qf(i)-q(i))*(qf(i)-q(i))^2;
% end
x=compute_hierarchy_sol(x, A, b, N);
N=compute_hierarchy_N(A, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max manipulability
% move along the null space
% compute the difference in the manipulability computed based on the 
% jacobians
%   Use sign_max=1 to maximize. sign_max = -1 to minimize.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x] = max_manipulability_simple(robot, q, sign_max)
%compute nil-space!!
J = manipulator_jacobian(robot, q);
I = eye(7);
Jp=penrose(J);
%null space projector
%n_space_projector = (I-Jp*J);
%for an arbitrary vector
%qd = [31 1 51 14 31 31 1]';
%qd = [1 1 1 1 1 1 1]';
qd = [0 0 1 0 0 0 0]';

%q2 está calculado a través de un proyector (I-Jp*J),
%, de tal manera que q2 pertenece al null space de J
qd_null = (I-Jp*J)*qd;
%normalize qd_null
qd_null = qd_null/norm(qd_null);

%movernos sobre qd_null no produce ningún movimiento en el extremo
% pero sí afecta a otros parámetros, como, por ejemplo, la manipulabilidad

%this is a numeric computation of manipulability differential
%a small positive differential movement along qd_null produces a change
% in manipulability
delta_manip = compute_delta_manip(robot, q, qd_null, 0.01);
%q = q + 10*sign_max*delta_manip*qd_null;

%compute restriction
%caution: both restrictions work the same way.
% we can just restrict the movement of q3 and compute the
%rest of self-motions.
%or, given that we already know the null space, we can just specify as A=I.
% I=qd_null as restriction is ok and returns the expected solution
%A = zeros(7,7);
%A(joint_i,joint_i) = 1;
%A = eye(7);

% % the restriction is then assigned so as to
% b = delta_manip*qd_null;
% 
% x=compute_hierarchy_sol(x, A, b, N);
%N=compute_hierarchy_N(A, N);
%based on that, the solution is just the null_space vector
%computed before.
%x = qd_null;
%but, add a constant for the step,
% add a sign for maximization/minimization
% the constant is based on the rate of delta_manip also
x = sign_max*sign(delta_manip)*qd_null;
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computes a gradient so as to let the manipulability be improved
% q is the current joint position
% qd is the instantaneous speed that lies in the null space
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_manip = compute_delta_manip(robot, q, qd, delta)
%compute initial Jacobian
J = manipulator_jacobian(robot, q);
%compute first manipulability index
m0 = det(J*J');
%move differentially along the null space.
q = q + delta*qd;
%compute another 
Jd = manipulator_jacobian(robot, q);
%compute second manipulability index
m1 = det(Jd*Jd');
%delta_manip = trace(inv(J*J')*(Jd*J'+J*Jd'));
%return difference
delta_manip=(m1-m0)/delta;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%compute atraction force to a specific solution
%force is proportional to distance!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function f = compute_forces(qf, q)
f=[];
for i=1:length(q)
    r = (qf(i)-q(i))^2;
    f(i)= sign(qf(i)-q(i))*r;
end
f = f(:);

function x = compute_hierarchy_sol(xim1, Ai, bi, Nim1)
x = xim1 + penrose(Ai*Nim1)*(bi-Ai*xim1);

function N = compute_hierarchy_N(Ai, Nim1)
I = eye(6); %identity matrix for the problem
N = Nim1*(I-penrose(Ai*Nim1)*(Ai*Nim1));

%compute moore penrose pseudo inverse
function P = penrose(B)
P = pinv(B);






