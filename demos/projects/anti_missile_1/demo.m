%
%   GANTLING ANTI-MISSILE SYSTEM SIMULATION
%   
%   LUIS DE LA OSSA DE PASCUAL & ANDRES DE LA OSSA DE PASCUAL
%
function demo()
    h = 0.001;
    timeMax = 50;
    t = 0:h:timeMax;
    Ra = 400; %Radio alcance
    Xm1 = [10000; 10000; 10000];% Primer punto del radar
    Xm2 = [9711.325; 9711.325; 9711.325];% Segundo punto del radar
    Dt = 1;
    
    %Calculamos la trayectoria del misil
    Vm = VelMisil(Xm1(1,:), Xm2(1,:), Xm1(2,:), Xm2(2,:), Xm1(3,:), Xm2(3,:), Dt);
    Tm = TrayMisil(Vm, Xm1(1,:), Xm1(2,:), Xm1(3,:), t);
   
    %Tomamos de punto objetivo un punto que esté a una distancia
    %suficientemente alejada pero en el que la trayectoria del proyectil
    %siga siendo moderadamente lineal
    [Pa, Ta] = DentroRango(Tm, Ra);
    
    %Calculamos todos los parametros que tendrá que tener el proyectil para
    %lograr alcanzar el punto objetivo en el tiempo correcto
    [theta, gamma, Xp, Timp] = think(Pa);
    
    f1 = figure('Name', 'Robot');
    f2 = figure('Name', 'Trayectoria proyectil y Simulación');
    
    figure(f1);
    view(-4,8);
    hold on;
    T = AngleRobot(theta, gamma);
    %qinv = inversekinematic(robot, T);
    %drawrobot3d(robot, qinv(:,7));
    
    
    figure(f2);
    subplot(1,2,1);
    view(-4,8);
    hold on;
    plot3(Xp(1, :), Xp(2, :), Xp(3, :));
    scatter3(Pa(1), Pa(2), Pa(3));
    
    subplot(1,2,2);
    anP = animatedline('Color', 'Red');
    anM = animatedline;
    hold on;
    axis([0 600 0 600 0 600]);
    scatter3(Pa(1), Pa(2), Pa(3));
    view(-4,8);
    hold on;
    Tsim = 0;
    a = tic;
    for i = 1:size(t,2)
        Tsim = Tsim + h;
        addpoints(anM, Tm(1, i), Tm(2, i), Tm(3, i))
        if Tsim >= Ta-Timp
            n = i-round((Ta-Timp)/h)+1;
            addpoints(anP , Xp(1, n), Xp(2, n), Xp(3, n))
            b = toc(a);
            if b > h/10;
                drawnow
                a = tic;
            end
        end
    end
end 


%Función que toma como valores dos puntos consecutivos en los que se ha
%detectado al misil y la diferencia de tiempo entre esas dos medidas y
%devuelve un vector velocidad del misil.
function Vm = VelMisil(x1, x2, y1, y2, z1, z2, Dt)
    Vm(1,1) = (x2 - x1)/Dt;
    Vm(2,1) = (y2 - y1)/Dt;
    Vm(3,1) = (z2 - z1)/Dt;
    
end

%Función que toma como valores la masa, la gravedad, la constante de
%rozamiento, y la velocidad actual del proyectil y devuelve su aceleración
function Ap = AcelProy(t, V, G, m, k)
    V2 = norm(V) * V;
    Ap = G - ((k/m) * V2);
end

%Función que toma como valores la velocidad del misil, la posición inicial
%y un vector tiempo y devuelve un vector con todas las posiciones que
%tendrá el misil para esos valores de tiempo.
function Tm = TrayMisil(Vm, x1, y1, z1, t)
    Tm = [; ; ; ];
    for tiempo = t
        Tm = [Tm [x1 + Vm(1) * tiempo; y1 + Vm(2) * tiempo; z1 + Vm(3) * tiempo; tiempo]];
    end
end


%Función que calcula el punto y el tiempo respectivo en el que el misil
%estará dentro del alcance del proyectil.
function [Pa, Ta] = DentroRango(Tm, Ra)
    for i = 1: size(Tm,2)
        if norm(Tm(1:3, i)) <= Ra
           Pa = Tm(1:3, i);
           Ta = Tm( 4 , i);
           break
        end
        Pa = -1000;
        Ta = -1000;
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


%Función que aplica Runge-Kutta a la función AcelProy y devuelve la
%velocidad y la posición del proyectil en el instante siguiente
function [Vp1,Xp1]  = RK4(tn, Xp, Vp, h, G, m, k)
    K1 = AcelProy(tn, Vp, G, m, k);
    K2 = AcelProy(tn + h/2, Vp + K1*h/2, G, m, k);
    K3 = AcelProy(tn + h/2, Vp + K2*h/2, G, m, k);
    K4 = AcelProy(tn + h, Vp + K3*h, G, m, k);
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


%Función que recibe como parámetro un punto objetivo (Po) y el instante en
%el que tiene que acertarle, y devuelve los ángulos theta y gamma en los
%que tiene que disparar el robot para acertarle (ver función anterior) y el
%instante en el que tiene que disparar
function [theta, gamma, Xp, Timp] = think(Po)
    Xi = [0; 0; 0];
    h = 0.001;
    G = [0; 0; -9.81];
    m = 0.269;
    k = 0.001;
    timeMax = 100;
    t = 0;
    Zant = Po(3, 1);
    
    
    theta = atan2(Po(2,1), Po(1,1)); %Asignamos una theta cualquiera (Primero debemos calcular gamma)
    d = sqrt(Po(1,1)^2+Po(2,1)^2); %Distancia horizontal (en el plano XY) desde el punto objetivo hasta el robot.   
    gamma = atan2(Po(3,1), d); %La primera gamma es como si fuera a ir en linea recta hasta el punto.
    disp('Primera gamma')
    disp(gamma)
    for a = 1:100
        Vp = VelProy(theta, gamma);
        Xp = RungeAlgoritmo(Xi, Vp, timeMax, t, h, G, m, k);
        impacto = 0;
        for p = Xp %Este bucle determina qué punto de la trayectoria llega al punto objetivo sin tener en cuenta la altura
            dd = sqrt(p(1,1)^2+p(2,1)^2);
            impacto = impacto + 1;
            if dd >= d
                Timp = impacto*h;
                break;
            end
        end
        
        Zobj = Po(3,1); %Z objetivo
        Zact = p(3,1); %Z actual
        
        if Zact >= (Zobj - 0.1) & Zact <= (Zobj + 0.1) %Si el proyectil está a la misma altura (con +-10 cm de error), la trayectoria es buena
            disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
            DIS = ["Gamma: ", gamma, "Error: ", abs(Zobj - Zact), "Alcanzado"];
            disp(DIS)
            p = p
            break;
        elseif Zact < (Zobj - 0.1)
            DIS = ["Gamma: ", gamma, "Error: ", Zobj - Zact, "Menor", "Nuevo objetivo: ", Zant + (Zobj - Zact)];
            disp(DIS)
            error = (Zobj - Zact);
            gamma = atan2(Zant + error, dd);
            Zant = Zant + ((Zobj - Zact));
        elseif Zact > (Zobj + 0.1)
            dis = ["Gamma: ", gamma, "Error: ", Zact - Zobj, "Mayor", "Nuevo objetivo: ", Zant + (Zact - Zobj)];
            disp(dis)
            error = (Zact - Zobj);
            gamma = atan2(Zant - error, dd);
            Zant = Zant - ((Zact - Zobj));
        end
        if a == 100 %Si ha realizado el número máximo de simulaciones y no ha encontrado el punto, emitimos un error
            dis("Punto no alcanzable")
        end
    end
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
    tRobot(3,1) = sin(gamma + pi/2) %xz
    
    tRobot(1,2) = cos(theta-pi/2); %yx
    tRobot(2,2) = sin(theta-pi/2); %yy
    tRobot(3,2) = 0; %yz
    
    tRobot(1, 4) = 0.0;
    tRobot(2, 4) = 0;
    tRobot(3, 4) = 0.9;
    tRobot(4, 4) = 1;
    
end




