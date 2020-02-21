%{
COMPONENTES DEL GRUPO:
- Mario Pérez Checa
- Jaime Sempere Ruiz
- Carlos Pina López
%}


%%%% Pasos previos a la ejecución de la demo:

   %% Introducir el valor deseado de xr (distancia de la base del robot a
    % la canasta, 4 metros por defecto) en la línea 70 del parameters.m
    % adjunto en la carpeta, sustituir dicho archivo por el parameters.m
    % del 3dofplanar de la librería ARTE (Es necesario sustituir, ya que
    % si únicamente se adjunta a la carpeta con otro nombre y se ejecuta no funciona.
    
   %% Cargar el robot 3dofplanar con el parameters.m modificado y elegir
    % los parámetros iniciales de la simulación: xr, teta, teta0 y wmax
    
   %% Ejecutar demo.m
robot = load_robot('example', '3dofplanar');
Proyecto
representacion