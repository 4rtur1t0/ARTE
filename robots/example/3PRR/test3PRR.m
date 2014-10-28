%-- script for the 3PRR --%

%   Authors: 
%           Rafael López Contreras
%           Francisco Martínez Femenía
%           Santiago Giménez García 
%           Albano López Gámez
%           Guillermo Salinas López
%           Jonatan Lloret Reina
%
%Universidad Miguel Hernández de Elche. 


T=eye(4);

%just try different positions and orientations.
T(1,4)=0.5;
T(2,4)=0.7;
phi=pi/10;
T(1,1)=cos(phi);
T(2,1)=sin(phi);
q = inversekinematic_3PRR(robot, T)


%init_lib
%disp('Introduce el valor de la variables activas d1, d2 y d3');
d1 = q(1,1); %input('Ingrese el valor de d1: ')
d2 = q(3,1);%input('Ingrese el valor de d2: ')
d3 = q(5,1);%input('Ingrese el valor de d3: ')
directa = direct_kinematics_3PRR_numerical(robot,[d1 d2 d3], 0.001)

disp('Se dibujan dos de las soluciones, comprobando que el efector final se encuentra en la misma posición y orientación');

%Se realiza un bucle for para dibujar las 8 posibles combinaciones.
for i=1 : 8
    directa = direct_kinematics_3PRR_numerical(robot,[q(1,i) q(3,i) q(5,i)], 0.001)
end
%Se observa en algunas que la solución no converge, por lo que la
%solución no será válida. Otras son soluciones reales, pero el efector no
%se mantiene en la posición/orientación deseada.