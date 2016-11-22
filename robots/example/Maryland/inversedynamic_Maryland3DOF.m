%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tau=inversedynamic_Maryland3DOF(robot, q, qdd, gc, fext)
%
% gc=9.81 if the robot is placed upwards
% gc=-9.81 if the robot is hanging.
%
% Consider a load of 10kg placed on the end effector. Write:
%   fext = [0 0 -10*9.81] if the robot is placed upwards, or
%   fext = [0 0 10*9.81] if the robot is hanging.
%
%   Author: Arturo Gil arturo.gil@umh.es
%   Date: 27/02/2014
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

function tau=inversedynamic_Maryland3DOF(robot, q, qdd, gc, fext)

fprintf('\nComputing inverse dynamics for the %s robot', robot.name);

fprintf('\nLagrangian Approach!!');

%forces expressed in the 0 reference system
fx = fext(1);
fy = fext(2);
fz = fext(3);

%obtain the active joints from the vector of generalized coordinates q
%this vector is defined in a Lagrangian style, thus
%q=[px, py, pz, th1 th2 th3]
% and qdd=[pxdd, pydd, pzdd, thdd1 thdd2 thdd3]
%redundant coordinates px py pz
px=q(1);
py=q(2);
pz=q(3);

%joint coordinates
th1=q(4);
th2=q(5);
th3=q(6);

%accelerations of the end effector
pxdd=qdd(1);
pydd=qdd(2);
pzdd=qdd(3); %m/s^2

%joint accelerations rad/s^2
th1dd=qdd(4);
th2dd=qdd(5);
th3dd=qdd(6);


%geometric parameters
rb=robot.rb;
re=robot.re;
L1=robot.L1;
L2=robot.L2;
Phi=robot.Phi;

%retrieve dynamic parameters
mp=robot.mp; %kg, mass of the end effector
mb=robot.mb; %kg, mass of each of the two beams that form the upper arm
%gc=9.81;
ma=robot.ma; %mass of the low arms

%taprox=(mp+3*mb)*0.55*gc/3+mb*0.3*gc+ma*gc*0.15;%*cos(1.82)

%taprox = ((mp/3+2*mb)*gc*0.3+ma*gc*0.15)*cos(th1)
% ff= (mp/3+mb)*gc*tan(acos(0.65/0.7));
% taprox=ff*0.3

%compute the displacements xi, yi zi
x1=(L1*cos(th1)+rb-re)*cos(Phi(1));
y1=(L1*cos(th1)+rb-re)*sin(Phi(1));
z1=L1*sin(th1);

x2=(L1*cos(th2)+rb-re)*cos(Phi(2));
y2=(L1*cos(th2)+rb-re)*sin(Phi(2));
z2=L1*sin(th2);

x3=(L1*cos(th3)+rb-re)*cos(Phi(3));
y3=(L1*cos(th3)+rb-re)*sin(Phi(3));
z3=L1*sin(th3);

%Just here to test that the constraint equations Gammai are correct
Gamma1=(px-x1)^2+(py-y1)^2+(pz-z1)^2-L2^2;
Gamma2=(px-x2)^2+(py-y2)^2+(pz-z2)^2-L2^2;
Gamma3=(px-x3)^2+(py-y3)^2+(pz-z3)^2-L2^2;

%compute the derivative of the displacements with respect to th1, th2 and
%th3
dx1=-L1*sin(th1)*cos(Phi(1));
dy1=-L1*sin(th1)*sin(Phi(1));
dz1=L1*cos(th1);

dx2=-L1*sin(th2)*cos(Phi(2));
dy2=-L1*sin(th2)*sin(Phi(2));
dz2=L1*cos(th2);

dx3=-L1*sin(th3)*cos(Phi(3));
dy3=-L1*sin(th3)*sin(Phi(3));
dz3=L1*cos(th3);

%Yes, now compute the first set of redundant equations
Nablaxyz = [2*(px-x1) 2*(px-x2) 2*(px-x3);
            2*(py-y1) 2*(py-y2) 2*(py-y3);
            2*(pz-z1) 2*(pz-z2) 2*(pz-z3)];
%Forces vector
F= [(mp+3*mb)*pxdd - fx;
    (mp+3*mb)*pydd - fy;
    (mp+3*mb)*pzdd + (mp+3*mb)*gc - fz];

%such that:
%
%Nabla*Lambda=F
%
%and Nablaxyz is the Jacobian of Gamma with respect to px, py and pz

%Solve Lagrange multipliers by inversion:
Lambda = inv(Nablaxyz)*F;
     

%Now compute torques using the previously computed Lagrange multipliers
%
%tau = L - Nablath*Lambda
%
%where Nablath is the Jacobian of Gamma with respect to the active coordinates

L=[((1/3)*ma*L1^2+mb*L1^2)*th1dd + ((1/2)*ma + mb)*cos(th1)*gc*L1;
   ((1/3)*ma*L1^2+mb*L1^2)*th2dd + ((1/2)*ma + mb)*cos(th2)*gc*L1;
   ((1/3)*ma*L1^2+mb*L1^2)*th3dd + ((1/2)*ma + mb)*cos(th3)*gc*L1];

Nablath = [-2*(px-x1)*dx1 - 2*(py-y1)*dy1-2*(pz-z1)*dz1        0 0;
            0       -2*(px-x2)*dx2 - 2*(py-y2)*dy2-2*(pz-z2)*dz2 0;
            0                  0     -2*(px-x3)*dx3 - 2*(py-y3)*dy3-2*(pz-z3)*dz3];

%Now compute the generalized torques at each joint
tau = L - Nablath*Lambda;

