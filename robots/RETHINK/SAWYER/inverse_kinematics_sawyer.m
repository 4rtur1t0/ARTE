
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
function q = inverse_kinematics_sawyer(robot, Tf, q0)
%global parameters
% try to maximize manipulabiity while moving the arm
if robot.maximize_manipulability == 1
    maximize_manipulability = 1;
elseif robot.maximize_manipulability == -1
    maximize_manipulability = -1;
else
    maximize_manipulability = 0;
end

% Obtain thea matriz de posición/orientación en Quaternion representation
Qf = T2quaternion(Tf);
Pf = Tf(1:3,4);
q=q0(:);
step_time = robot.parameters.step_time;
i=0;

%this is a gradient descent solution based on moore-penrose inverse
while i < robot.parameters.stop_iterations
    Ti = directkinematic(robot, q);
    Qi = T2quaternion(Ti);
    Pi = Ti(1:3,4);
   
    eps1 = reached_position(Pf, Pi);
    eps2 = reached_orientation(Qf, Qi);
    %compute linear speed and angular speed that are served as a high level
    %based on the current pose
    v0 = compute_high_level_action_kinematic_v(Pf, Pi); %1m/s 
    w0 = compute_high_level_action_kinematic_w(Qf, Qi); %1rad/s
    %the restriction is the speed to reach the point
    Vref = [v0' w0']';
    if eps1 < robot.parameters.epsilonXYZ && eps2 < robot.parameters.epsilonQ
        fprintf('INVERSE KINEMATICS SUCCESS: REACHED epsilonXYZ AND epsilonQ\n')
        q = atan2(sin(q), cos(q));
        return;
    end
    %qd = inverse_kinematic_moore_penrose(robot, q, Vref);
    J = manipulator_jacobian(robot, q);
    iJ = inverse_jacobian(J, 'dampedmoore');
    qd = iJ*Vref;
    % yes, normalize speed to avoid inconsistencies
    % qd is normalize so as to have higher Vref
    qd = normalize_qd(qd, norm(Vref));

    % add a null movement to maximize manipulability
    if maximize_manipulability==1 ||  maximize_manipulability==-1  
        % caution, if maximize_manipulability = +1--> maximizes manip.
        % else if maximize_manipulability = -1 it is minimized
        qdm = max_manipulability_simple(robot, q, maximize_manipulability);
        qdm = qdm/sum(abs(qdm));
        qd = 0.5*qd + 0.5*qdm;
    end
    %actually move the robot.
    q = q + qd*step_time;
    % drawrobot3d(robot, q)
    % pause(0.1)
    i=i+1;
    eps1
    eps2
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

%
% Normalize qd based on the current error in position and orientation.
%
function qd = normalize_qd(qd, e)
% qdmax = max(abs(qd));
%normalize to unit vector
qd = qd/abs(sum(qd));
x = [0.001 0.01 0.1 0.5 1 2 3 5];
v = [0.001 1  5 10 20 20 20 20];
kp = interp1(x,v,e);
% finally scale
qd = kp*qd;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check whether orientation has been reached
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reach = reached_orientation(Qf, Qi)
Q = Qf-Qi;
reach = sqrt(Q(1)^2 + Q(2)^2 + Q(3)^2 + Q(4)^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check whether orientation has been reached
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reach = reached_position(Pf, Pi)
P = Pf-Pi;
reach = sqrt(P(1)^2 + P(2)^2 + P(3)^2);


function qd = inverse_kinematic_moore_penrose(robot, q, Vref)
J = manipulator_jacobian(robot, q);
Jp = pinv(J);
qd = Jp*Vref;



function [x] = max_manipulability_simple(robot, q, sign_max)
%compute nil-space!!
J = manipulator_jacobian(robot, q);
I = eye(7);
Jp = pinv(J);
%null space projector
qd = [0 0 1 0 0 0 0]';
qd = q; % [0 0 1 0 0 0 0]';

%q2 está calculado a través de un proyector (I-Jp*J),
%, de tal manera que q2 pertenece al null space de J
qd_null = (I-Jp*J)*qd;
%normalize qd_null
qd_null = qd_null/sum(abs(qd_null)); %norm(qd_null);

% in manipulability
delta_manip = compute_delta_manip(robot, q, qd_null, 0.01);

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
