function T = directkinematic_1dofplanar(robot, q)


if isfield(robot, 'parallel')
    T = eval(robot.directkinematic_fn);
    return;
end

%Cinemática directa de cada brazo
theta = q(1);
d = 0;
a = d(1);
alfa = 0;

n=length(theta); %número de GDL

if robot.debug
    fprintf('\nComputing direct kinematics for the %s robot with %d DOFs\n',robot.name, n);
end
%Cargar la posición/orientación de la base del robot

T = robot.T0;

for i=1:n,
    T=T*dh(theta(i), d(i), a(i), alfa(i));    
end
