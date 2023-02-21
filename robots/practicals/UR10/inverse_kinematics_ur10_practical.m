
%   INVERSE KINEMATICS FOR THE UR10 ROBOT
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
function [q] = inverse_kinematics_ur10_practical(robot, Tf, q0)
step_time = 0.1; 
max_error= 0.00001;
max_iterations = 1000;

% Obtain thea matriz de posici�n/orientaci�n en Quaternion representation
Qf = T2quaternion(Tf);
Pf = Tf(1:3,4);
q=q0(:);
i=0;
error = [];
qs = [q0];
qds = [];

while i < max_iterations
    fprintf('\nIteration %f', i)
    % current homogeneous matrix T
    Ti = directkinematic(robot, q);
    % Current orientation given by a quaternion
    Qi = T2quaternion(Ti);
    Pi = Ti(1:3,4);
    %compute linear speed and angular speed that are served as a high level
    %based on the current pose
    v0 = compute_high_level_action_kinematic_v(Pf, Pi); %1m/s Pf - Pi
    w0 = compute_high_level_action_kinematic_w(Qf, Qi); %1rad/s
    Vref = [v0' w0']';
    
    if norm(Vref) < max_error
        break;
    end
    % JACOBIANA DEL ROBOT
    J = manipulator_jacobian(robot, q);   
    if det(J) == 0
        qd = rand(6, 1);
    else
        qd = inv(J)*Vref;     
    end
        % CINEMÁTICA INVERSA
    %qd = inv(J)*Vref;    
    
    % CALCULE LA PROXIMA ITERACION DEL ALGORITMO
    q = q + step_time*qd;
    qs = [qs q];
    qds = [qds qd];    
    error = [error norm(Vref)];
    i=i+1;
    %drawrobot3d(robot, q)
    %pause(0.01);
end

q = atan2(sin(q), cos(q));

if norm(Vref) < 0.00001
        fprintf('INVERSE KINEMATICS SUCCESS: REACHED epsilonXYZ AND epsilonQ\n')
else
    fprintf('\n\nINVERSE KINEMATICS FAILED: COULD NOT REACH POSITION/ORIENTATION\n')
end

animate(robot, qs);
close all,
figure, plot(error),
figure, plot(qs')
figure, plot(qds')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posici�n y orientaci�n final, calcular cu�les deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v] = compute_high_level_action_kinematic_v(Pf, Pi)
%compute a constant linear speed till target
v = (Pf-Pi);
v = v(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posici�n y orientaci�n final, calcular cu�les deben ser las
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



