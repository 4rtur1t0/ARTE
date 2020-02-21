%Simulación únicamente del robot, introducir un punto objetivo en Po
%para probar el algoritmo. El extremo del robot apuntará en la dirección
%necesaria para acertar en el misil.


function Simulacion_robot(robot)
    Po = [650; 650; 650]; %%%%%%%%%%%%%%%%PUNTO OBJETIVO%%%%%%%%%%%%%%%%%%%%%%%%5
    [theta, gamma, Xp] = think(Po);
    T = AngleRobot(theta, gamma);
    qinv = inversekinematic(robot, T);
    drawrobot3d(robot, qinv(:,7));
end


%Función que toma como valores la masa, la gravedad, la constante de
%rozamiento, y la velocidad actual del proyectil y devuelve su aceleración
function Ap = AcelProy(t, V, G, m, k)
    kmv = k/m * norm(V) * V;
    Ap = G - kmv;
end

%Función que aplica Runge-Kutta a la función AcelProy y devuelve la
%velocidad y la posición del proyectil en el instante siguiente
function [Vp1,Xp1]  = RK4(tn, Xp, Vp, h, G, m, k)
    K1 = AcelProy(tn, Vp, G, m, k);
    K2 = AcelProy(tn + h/2, Vp + K1*h/2, G, m, k);
    K3 = AcelProy(tn + h/2, Vp + K2*h/2, G, m, k);
    K4 = AcelProy(tn + h  , Vp + K3*h, G, m, k);
    Vp1 = Vp + (K1 + 2*K2 + 2*K3 + K4)*(h/6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    K1x = Vp;
    K2x = Vp + K1x*h/2;
    K3x = Vp + K2x*h/2;
    K4x = Vp + K3x*h;
    Xp1 = Xp + (K1x + 2*K2x + 2*K3x + K4x)*(h/6);
end

%Función que realiza el algoritmo de Runge Kutta y devuelve la trayectoria
%del proyectil
function Xp = RungeAlgoritmo(Xp, Vp, timeMax, t, h, G, m, k)
    while (t < timeMax) & ( Xp(3,length(Xp(1,:))) >= 0 ) %Para cuando el proyectil toca el suelo o cuando se alcanza el límite de tiempo
        [RungeV, RungeX] =  RK4(t, Xp(:,length(Xp(1,:))), Vp(:,length(Vp(1,:))), h, G, m, k);
        Vp = [Vp RungeV];
        Xp = [Xp RungeX];
        t = t + h;
    end
end

%Función que recibe los ángulos theta y gamma sobre los que tiene que
%disparar y calcula las componentes x, y y z de la velocidad para que el
%módulo de la velocidad sea de 1035 m/s
%Theta : Ángulo que forman la proyección del vector velocidad en el plano
%   XY y el eje X.
%Gamma : Ángulo que forma el vector velocidad con el eje Z.
function V = VelProy(theta, gamma)
    V(1,1) = 1035 * cos(gamma) * cos(theta);
    V(2,1) = 1035 * cos(gamma) * sin(theta);
    V(3,1) = 1035 * sin(gamma);
end

%Función que recibe en ángulo theta y gamma a los que debe apuntar el eje z
%de la muñeca del robot y devuelve una matriz T
function tRobot = AngleRobot(theta, gamma)
    tRobot = zeros(4, 4);
    tRobot(1,3) = cos(gamma) * cos(theta); %zx
    tRobot(2,3) = cos(gamma) * sin(theta); %zy
    tRobot(3,3) = sin(gamma); %zz
    
    tRobot(1,1) = cos(gamma + pi/2) * cos(theta + pi); %xx
    tRobot(2,1) = cos(gamma + pi/2) * sin(theta + pi); %xy
    tRobot(3,1) = sin(gamma + pi/2); %xz
    
    tRobot(1,2) = cos(theta-pi/2); %yx
    tRobot(2,2) = sin(theta-pi/2); %yy
    tRobot(3,2) = 0; %yz
    
    tRobot(1, 4) = 0.0;
    tRobot(2, 4) = 0;
    tRobot(3, 4) = 0.9;
    tRobot(4, 4) = 1;
    
end

%Función que recibe como parámetro un punto objetivo (Po), y devuelve los ángulos theta y gamma en los
%que tiene que disparar el robot para acertarle (ver función anterior) y la
%trayectoria que va a seguir la bala.
function [theta, gamma, Xp] = think(Po)
    Xi = [0; 0; 0];
    h = 0.001;
    G = [0; 0; -9.81];
    m = 0.269;
    k = 0.0005;
    timeMax = 100;
    t = 0;
    Zant = Po(3, 1); %Z ulitizado en la iteración anterior 
    
    theta = atan2(Po(2,1), Po(1,1)); %Asignamos una theta cualquiera (Primero debemos calcular gamma)
    d = sqrt(Po(1,1)^2+Po(2,1)^2); %Distancia horizontal (en el plano XY) desde el punto objetivo hasta el robot.   
    gamma = atan2(Po(3,1), d); %La primera gamma es como si fuera a ir en linea recta hasta el punto.

    for a = 1:100
        Vp = VelProy(theta, gamma);
        Xp = RungeAlgoritmo(Xi, Vp, timeMax, t, h, G, m, k);
        
        for p = Xp %Este bucle determina qué punto de la trayectoria llega al punto objetivo sin tener en cuenta la altura
            dd = sqrt(p(1,1)^2+p(2,1)^2);
            if dd >= d
                break;
            end
        end
        
        Zobj = Po(3,1); %Z objetivo
        Zact = p(3,1); %Z actual
        
        if Zact >= (Zobj - 0.1) & Zact <= (Zobj + 0.1) %Si el proyectil está a la misma altura (con +-10 cm de error), la trayectoria es buena
            break;
        elseif Zact < (Zobj - 0.1)
            error = (Zobj - Zact);
            gamma = atan2(Zant + error, dd);
            Zant = Zant + ((Zobj - Zact));
        elseif Zact > (Zobj + 0.1)
            error = (Zact - Zobj);
            gamma = atan2(Zant - error, dd);
            Zant = Zant - ((Zact - Zobj));
        end
    end
end