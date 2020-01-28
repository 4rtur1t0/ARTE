
%   INVERSE KINEMATICS FOR THE SAWYER ROBOT
%
%   Solves the inverse kinematic problem in various situations.
%   A Jacobian based method is used.
%   The method tries to reach the given position/orientation while, at the 
%   same time, maximizing/minimizing a secondary target.
%
%   e.g. reach position/orientation and maximize manipulability det(J'J)
%   
%   q0: starting initial solution
%   Tf--> final position/orientation wanted as a homogeneous matrix
function q = inverse_kinematics_ur10_practical(robot, Tf, q0)
% Obtain thea matriz de posición/orientación en Quaternion representation
Qf = T2quaternion(Tf);
Pf = Tf(1:3,4);
q=q0(:);
step_time = robot.parameters.step_time;
i=0;

%this is a gradient descent solution based on moore-penrose inverse
while i < robot.parameters.stop_iterations
    fprintf('Iteration %f', i)
    % current homogeneous matrix T
    Ti = directkinematic(robot, q);
    % Current orientation given by a quaternion
    Qi = T2quaternion(Ti);
    Pi = Ti(1:3,4);
    %compute linear speed and angular speed that are served as a high level
    %based on the current pose
    v0 = compute_high_level_action_kinematic_v(Pf, Pi); %1m/s 
    w0 = compute_high_level_action_kinematic_w(Qf, Qi); %1rad/s
    Vref = [v0' w0']';
    % Acciones de control de alto nivel
    nv0=norm(v0)
    nw0=norm(w0)
    if norm(Vref) < 0.001
        fprintf('INVERSE KINEMATICS SUCCESS: REACHED epsilonXYZ AND epsilonQ\n')
        q = atan2(sin(q), cos(q));
        return;
    end
    % Calcule, a continuación, la JACOBIANA DEL ROBOT
    J = manipulator_jacobian(robot, q);
    % EN BASE A LA JACOBIANA, CALCULE UNA ACCIÓN DE ACTUALIZACIÓN qd 
    % USANDO LA JACOBIANA INVERSA
    % USANDO LA JACOBIANA TRANSPUESTA

    
    
    % CALCULE LA PRÓXIMA ITERACIÓN DEL ALGORITMO
    q = q + qd*step_time;
    drawrobot3d(robot, q)
    i=i+1;
end
fprintf('INVERSE KINEMATICS FAILED: COULD NOT REACH POSITION/ORIENTATION\n')

q = atan2(sin(q), cos(q));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posición y orientación final, calcular cuáles deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v] = compute_high_level_action_kinematic_v(Pf, Pi)
%compute a constant linear speed till target
v = (Pf-Pi);
v = v(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posición y orientación final, calcular cuáles deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [w] = compute_high_level_action_kinematic_w(Qf, Qi)
%compute a constant angular speed till target
%asume the movement is performed in 1 second
w = angular_w_between_quaternions(Qi, Qf, 1);
w = w(:);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute angular speed w that moves Q0 into Q1 in time total_time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = angular_w_between_quaternions(Q0, Q1, total_time)
%global robot
%below this number, the axis is considered as [1 0 0]
%this is to avoid numerical errors
%this is the actual error allowed for w
%epsilon_len = robot.parameters.epsilonQ;
epsilon_len = 0.0001;
%Let's first find quaternion q so q*q0=q1 it is q=q1/q0 
%For unit length quaternions, you can use q=q1*Conj(q0)
Q = qprod(Q1, qconj(Q0));

%To find rotation velocity that turns by q during time Dt you need to 
%convert quaternion to axis angle using something like this:
len=sqrt(Q(2)^2 + Q(3)^2 + Q(4)^2);

if len > epsilon_len
    angle=2*atan2(len, Q(1));
    axis=[Q(2) Q(3) Q(4)]./len;
else
    angle=0;
    axis=[1 0 0];
end
w=axis*angle/total_time;









function qd = inverse_kinematic_moore_penrose(robot, q, Vref)
J = manipulator_jacobian(robot, q);
Jp = pinv(J);
qd = Jp*Vref;



