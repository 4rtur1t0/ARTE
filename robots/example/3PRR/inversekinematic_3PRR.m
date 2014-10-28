%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the 3PRR planar parallel robot
% The 3RRR robot is divided into three different 2DOF arms.
%
% INVERSEKINEMATIC_3PRR(robot, T)
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions for q = [Q1 Q2 Q3 Q4 Q5 Q6] 
% that place the end effectors at the position specified by T. 
% Any Qi is a 4 value vector that contains: Qi={q1 fi1, q2, fi2, q3, fi3},
% being q1, fi1, the joint variables of the first arm
%  and  q2, fi2, the joint variables of the second arm
%  and  q3, fi3, the joint variables of the second arm
%
% Eight possible solutions q = [Q1 Q2 Q3 Q4 Q5 Q6],
% generally called elbow up and elbow down in eight different combinations for
% each of the three arms, as described in the following table:
%  
%               Arm 1       Arm 2      Arm 3
%   Sol 1        0           0          0   
%   Sol 2        0           0          1
%   Sol 3        0           1          0
%   Sol 4        0           1          1
%   Sol 5        1           0          0
%   Sol 6        1           0          1
%   Sol 7        1           1          0
%   Sol 8        1           1          1
%
%       1: elbow up solution
%       0: elbow down solution
%   Authors: 
%           Rafael López Contreras
%           Francisco Martínez Femenía
%           Santiago Giménez García 
%           Albano López Gámez
%           Guillermo Salinas López
%           Jonatan Lloret Reina
%
%Universidad Miguel Hernandez de Elche. 
%   Date: 10/02/2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function Q = inversekinematic_3PRR(robot, T)

%variable global para los eslabones que no varian su distancia
global b1

%position of the fixed points xQ, yQ, xR, yR
xQ=1; yQ=0; xR=0; yR=1.84;

% Geometry
b1=1; b2=1; b3=1;
%the end effector is a triangle of side h
h=0.5;

%Now find the orientation from the angle phi encoded in T=(ux, uy, uz)
u=T(1:3,1);
phi=atan2(u(2), u(1));

%Position of the point A in base coordinates
xA=T(1,4);
yA=T(2,4);

%Position of the point B in base coordinates
xB=xA+h*cos(phi);
yB=yA+h*sin(phi);

%Position of the point C in base coordinates
xC=xA+h*cos(phi+pi/3);
yC=yA+h*sin(phi+pi/3);

%transform to the Q reference system
xB_Q=xB-xQ;
yB_Q=yB-yQ;

%transform to the R reference system
xC_R=xC-xR;
yC_R=yC-yR;

%Soluciones de la inversa de cada brazo llamando a la función "calcula"

[d1, alpha1]=calcula(xA,yA);
[d2, alpha2]=calcula(xB_Q,yB_Q);
[d3, alpha3]=calcula(xC_R,yC_R);

%Q=[d1 d2 d3 alpha1 alpha2 alpha3];
%A continuación se muestran las 8 soluciones posibles:
% Q=[ d1(1) alpha1(1) d2(1) alpha2(1) d3(1) alpha3(1);
%     d1(1) alpha1(1) d2(1) alpha2(1) d3(2) alpha3(2);
%     d1(1) alpha1(1) d2(2) alpha2(2) d3(1) alpha3(1);
%     d1(1) alpha1(1) d2(2) alpha2(2) d3(2) alpha3(2);
%     d1(2) alpha1(2) d2(1) alpha2(1) d3(1) alpha3(1);
%     d1(2) alpha1(2) d2(1) alpha2(1) d3(2) alpha3(2);
%     d1(2) alpha1(2) d2(2) alpha2(2) d3(1) alpha3(1);
%     d1(2) alpha1(2) d2(2) alpha2(2) d3(2) alpha3(2)];

%q1=[d1(1) alpha1(1)];

Q=[  d1(1)    d1(1) d1(1) d1(1) d1(2) d1(2) d1(2) d1(2);
   alpha1(1)  alpha1(1)  alpha1(1)  alpha1(1) alpha1(2) alpha1(2)  alpha1(2)  alpha1(2) ;
     d2(1) d2(1) d2(2) d2(2)   d2(1) d2(1) d2(2) d2(2);
   alpha2(1) alpha2(1) alpha2(2) alpha2(2) alpha2(1) alpha2(1) alpha2(2) alpha2(2)                    ;
     d3(1)    d3(2) d3(1)    d3(2) d3(1)    d3(2) d3(1)    d3(2);
   alpha3(1)  alpha3(2) alpha3(1)  alpha3(2) alpha3(1)  alpha3(2) alpha3(1)  alpha3(2)];


%Función para resolver la cinemática inversa de cada brazo del robot 3PRR,
%con las dos soluciones para cada uno.

function [d,alpha]=calcula(x,y)
global b1
alpha(1)=asin(y/b1);
alpha(2)=pi-asin(y/b1);
d(1)=x-b1*cos(alpha(1));
d(2)=x+b1*cos(alpha(1));    %Porque cos(180-a)= - cos(a)