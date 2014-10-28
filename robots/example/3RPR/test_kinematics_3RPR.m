% Mecanismo 3RPR
% 4º Grado Ingeniería Electrónica y Automática Industrial
% Asignatura: Robótica
% 2013/2014

% Merlos Ortega, Juan Antonio
% Pérez Sotobal, Enrique




%function test_kinematics_3RPR(robot)


%Cálculo de la cinemática directa

%Utilizamos el algoritmo de Gauss-Newton, con este método obtenemos
%una de las posibles soluciones. De esta forma, se devuelve una 
%matriz T que contiene el punto A(Px,Py,0) del efector final y la
%orientación del mismo.
T=eye(4);

T(1,4)=1.1;
T(2,4)=1.2;

q=inversekinematic_3RPR(robot,T)

T=directkinematic_3RPR_numerical(robot,[q(1) q(3) q(5)])

%Se ha elegida esta posicion cualquiera, los mismos pueden ser variados.

%Al ejecutar la funcion nos devuelve lo siguiente:
%directkinematic_3RPR::Solucion encontrada en 3 iteraciones
%A tener en cuenta que solo se devuelve una de las posibles soluciones
%T =

%    0.9385    0.3453         0    0.9510
%   -0.3453    0.9385         0    0.9792
%         0         0    1.0000         0
%         0         0         0    1.0000



%Donde vemos que nos da la solución Px=0.9510, Py=0.9792 para el punto A.



%Cálculo de la cinemática inversa


%Al calcular la inversa, realizamos a su vez la llamada a otra función
%auxiliar que nos calcula la inversa de cada brazo de 2GDL RP.
%La funcion nos devuelve una matriz q que contiene una única solución
%posible, de forma que
%             q =
%   Brazo 1      q(1)  
%                q(2)  
%   Brazo 2      q(1)  
%                q(2)   
%   Brazo 3      q(1)  
%                q(2)  
%   
%Siendo q(1)los ángulos girados en los orígenes 
%y q(2) las longitudes de los brazos.
%de forma que:

%Calculando la cinematica inversa para el robot 3RPR parallel
%Calculando la cinematica inversa del robot 2DOF planar arm
%Calculando la cinematica inversa del robot 2DOF planar arm
%Calculando la cinematica inversa del robot 2DOF planar arm
%q =

 %   0.8000 = q(1) brazo 1 OK!
 %   0.3650
 %   2.5000 = q(1) brazo 2 OK!
 %   0.3477
 %  -1.5000 = q(1) brazo 3 OK!
 %   0.2037

 
%Sumando las variables articulares q(2) a la longitud de la parte 
%no variable del brazo (a(1)), tendremos la longitud total del brazo
%en cada posición. Para este caso:

%Brazo 1: 0.3650 + 1= 1.3650 metros
%Brazo 2: 0.3477 + 1= 1.3477 metros
%Brazo 3: 0.2037 + 1= 1.2037 metros

