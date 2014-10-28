% Mecanismo 3RPR
% 4º Grado Ingeniería Electrónica y Automática Industrial
% Asignatura: Robótica
% 2013/2014

% Merlos Ortega, Juan Antonio
% Pérez Sotobal, Enrique

function q = inversekinematic_2dofplanarRP(robot, T)

fprintf('\nCalculando la cinematica inversa del robot %s', robot.name);

%Inicializamos q
q=zeros(2,2);

% theta = eval(robot.DH.theta);
 d = eval(robot.DH.d);
 a = eval(robot.DH.a);
% alpha = eval(robot.DH.alpha);

Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Distancia entre el punto y el origen del sistema = Longitud del brazo
r= sqrt(Px^2+Py^2);

alpha= real(atan(Py/Px));


if    Px<0&&Py>=0
        tetha=alpha+pi;    
elseif Px<0&&Py<0
        tetha=alpha-pi;
else
        tetha=alpha;
end

q =[tetha;
    r-a(1)];


