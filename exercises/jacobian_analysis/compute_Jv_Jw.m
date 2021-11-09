
% Compute Jw
%robot = load_robot
q = [0 0 0 0 0 0]';
qd = [1 1 1 1 1 1]';
z0 = [0 0 1]';
% Init Jw
Jw = [z0];

A = eye(4);

% compute z1...z5
for i=1:5
   Ai = dh(robot, q, i);
   A=A*Ai;
   zi = A(1:3,3);
   Jw = [Jw zi];    
end

Jw
w = Jw*qd

% Compute Jv
T = directkinematic(robot,q);
p06 = T(1:3,4);
Jv = [cross(z0,p06)];
A = eye(4);
% compute z1...z5
for i=1:5
   Ai = dh(robot, q, i);
   A=A*Ai;
   p_i=A(1:3,4);
   zi = A(1:3,3);
   p = p06-p_i;
   Jv = [Jv cross(zi,p)];    
end
Jv 
v = Jv*qd

J = [Jv; Jw]

Jm = manipulator_jacobian(robot, q)
Jm