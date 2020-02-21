function [q,qd,v,w,time,T]=demo()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% El proyecto se ha elaborado empleando el Robot KUKA KR160_R1570_nanoC 
% Este robot debera estar añadido en la libreria de arte, en la carpeta KUKA
%
% Variables de diseño:
%     intervalos                -- Puntos que dibujamos desde origen hasta inicio dibujo y
%                                  del final al origen
%     interpolacion             -- Puntos trayectoria para reposicionarse mientras dibuja
%     escala                    -- Tamaño de la letra que queremos dibujar (Tamaño en plano, 
%                                  al pasarlo a la habrá deformación)en metros
%     radio                     -- Radio de la esfera sobre la que queremos dibujar (en metros)
%     multipuntos               -- Cantidad puntos del dibujo en la esfera
%     palabra                   -- Introducimos las letras a dibujar en la esfera 
%     Tinicial                  -- Posicion inicial del robot.
%     velocidad                 -- Velocidad constante a la que se
%                                  desplazará el actuador (cm/s)
%     precision                 -- Distancia máxima desde la esfera al
%                                  actuador (m)
%     delta_t                   -- Tiempo entre posición y posición en trayectoria.m 
%     mode                      -- Modo de la simulacion : 
%                                       +letras : dibuja solo las letras en la esfera segun la                                      
%                                        proximidad del efector (no tiene mucha precision la imagen ya que  
%                                        plotea los puntos como si fuesen sucesivos)
%                                       +trayectoria : dibuja la trayectoria recorrida
%
%     gx                        -- Giro eje x esfera en radianes
%     gy                        -- Giro eje y esfera en radianes
%     gz                        -- Giro eje z esfera en radianes
%     pos                       -- Posición esfera en metros
%     y                         -- Podemos cambiar el orden de las matrices,
%                                  para cambiar la rotación de la esfera y orientarla como deseamos
%
% Funciones:
%     Representacion_en_esfera  -- Dibuja la esfera deseada
%     path_planning              -- Genera el camino a seguir por robot
%     en_esfera                 -- Transformación del plano a la esfera 
%     Multipunto_interpolacion  -- Saca los puntos de la trayectoria de la escritura
%     paint_robot               -- Genera las matrices T de cada punto 
%     simular                   -- Pinta letras sobre esfera 
%     trayectoria               -- Genera el path planning final, con las
%                                  velocidades y ángulos en cada instante
%     inicial                   -- Genera la trayectoria desde la posicion
%                                  inicial hasta el primer punto de pintado
%                                  de la esfera
%     final                     -- Genera la trayectoria desde la posicion
%                                  final del pintado hasta el punto inicial
%                                  
%      Autores: 
%           Abiel Carrión Bailén    
%           Pablo Vicente Vidal      
%           Ramón Fontelles Congost  
%   Universitas Miguel Hernandez, SPAIN.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global palabra;
global multipuntos;
global radio;
global escala;
global interpolacion;
global intervalos;
global Tinicial;
global velocidad;
global precision;
global delta_t;

intervalos=5;
interpolacion=3;
escala=0.15;
radio=0.23;
multipuntos=3;
palabra='AM I A ROBOT?';
robot=load_robot('KUKA','KR160_R1570_nanoC'); %Cargamos robot
Tinicial=directkinematic(robot,[0 0 0 0 0 0]);
velocidad=5;        % cm/s
precision=0.001;    %Aproximacion a la esfera.
mode='letras';      %trayectoria o letras

delta_t=1;          %Recomendable valores bajos (0.01 o menos), pero para simular alarga mucho el tiempo de simulacion.
                    %Se puede implementar la funcion trayectoria.m a parte para
                    %sacar los valores. (s)
                    
    gx=pi/2;          % En radianes
    gy=0;
    gz=pi;
    pos=[0,-1,1]; % En metros
    
    Tx=[1 0 0 0; 0 cos(gx) -sin(gx) 0; 0 sin(gx) cos(gx) 0; 0 0 0 1 ];
    Ty=[cos(gy) 0 sin(gy) 0; 0 1 0 0;-sin(gy) 0 cos(gy) 0; 0 0 0 1 ];
    Tz=[cos(gz) -sin(gz) 0 0; sin(gz) cos(gz) 0 0; 0 0 1 0; 0 0 0 1 ];
    Tp=[1 0 0 pos(1); 0 1 0 pos(2); 0 0 1 pos(3); 0 0 0 1 ];  
    y=Tp*Tx*Ty*Tz;
    
tt=paint_robot;

T=simular(robot,tt,y,mode);    %%% T de los puntos base por los que pasa la trayectoria (simulados)

[q,qd,v,w,time]=trayectoria(T,robot);  %%% valores durante toda la trayectoria
end