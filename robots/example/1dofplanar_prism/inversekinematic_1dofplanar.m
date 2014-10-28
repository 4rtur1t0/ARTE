
function q = inversekinematic_1dofplanar(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);



q=zeros(2,1);

% theta = eval(robot.DH.theta);
 d = eval(robot.DH.d);
 a = eval(robot.DH.a);
% alpha = eval(robot.DH.alpha);

Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Distancia entre el punto y el origen del sistema
R= sqrt(Px^2+Py^2);

%Longitud del brazo
L1=abs(a(1));

if R > (L1)
   fprintf('\nERROR: inversekinematic_2dofplanar: unfeasible solution'); 
end


L1=R;
gamma= real(asin(Py/L1));

q =[pi-gamma;
    pi+gamma];

