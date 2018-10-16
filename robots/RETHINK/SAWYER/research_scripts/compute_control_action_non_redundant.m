%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First trial to compute just inverse kinematics using a square J and a
% gradient descent technique.
%
% Compute a solutions for V = J*qd
% find the joint speeds given V = [v w]
% v and w are the speeds of the end effector specified in 
% the base reference system.
% joint 0 is the joint set to zero in the solution.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vq = compute_control_action_non_redundant(robot, q, V, joint_0)
%vq = invert(robot, q, V, joint_0);
vq = invertLM(robot, q, V, joint_0);
%vq = [vq(1:4); 10*vq(5:7)]

function vq = invert(robot, q, V, joint_0)
J = manipulator_jacobian(robot, q);  
%Get J squared, remove redundant column
J(:,joint_0)=[];
%assure V is a column vector
V = V(:);
%Solve inverse kinematic problem with the inversion of J
vq = J\V;%inv(J)*V';
%return the whole solution
vq = [vq(1:joint_0-1); 0; vq(joint_0:end)];

function vq = invertLM(robot, q, V, joint_0)
J = manipulator_jacobian(robot, q);  
%Get J squared, remove redundant column
J(:,joint_0)=[];
lambda=1;
%assure V is a column vector
V = V(:);
vq=inv(J'*J + lambda^2*eye(6))*J'*V;

%return the whole solution
vq = [vq(1:joint_0-1); 0; vq(joint_0:end)];


