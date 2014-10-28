%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [JX, Jq, Jphi]=compute_jacobians_5R(robot, q)
%
%   Compute Jacobians for the for the 5R planar parallel robot
%
%   JX is the 4x2 Jacobian with respect to the cartesian coordinates (x, y)
%   Jq is the 4x2 Jacobian with respect to the active joints (q1, q2)
%   Jphi is the 4x2 Jacobian with respect to the passive joints (phi1, phi2)
%
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 13/09/2013
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

function [JX, Jq, Jphi]=compute_jacobians_5R(robot, q)

% The geometric parameters of this robot can be retrieved from 
% the two DOF planar arms that form it
a1=eval(robot.robot1.DH.a);
a2=eval(robot.robot2.DH.a);
 
%Link lengths
l1=abs(a1(1));
l2=abs(a1(2));
l3=abs(a2(1)); 
l4=abs(a2(2));
%distance along X that separate both arms
L=robot.L;

%retrieve the active and passive joints from the vector q
q1=q(1);
phi1=q(2);
q2=q(3);
phi2=q(4);

%The loop closure equations for this robot can be written as:
%
% \begin{bmatrix}
% \Gamma_1\\
% \Gamma_2\\
% \Gamma_3\\
% \Gamma_4
% \end{bmatrix}=
% \begin{bmatrix}
% x-l_1\cos(q_1)-l_2\cos(q_1+\varphi_1)\\
% y-l_1\sin(q_1)-l_2\sin(q_1+\varphi_1)\\
% x-l_4\cos(q_2)-l_3\cos(q_2+\varphi_2)-L\\
% y-l_4\sin(q_2)-l_3\sin(q_2+\varphi_2)
% \end{bmatrix}

%In this matrix, JX11 is the partial derivative of Gamma1 with respect to x
%                JX12 is the partial derivative of Gamma1 with respect to y
%                JX21 is the partial derivative of Gamma2 with respect to x
%...
JX=[1   0;
    0   1;
    1   0;
    0   1];

%In this matrix, Jq11 is the partial der. of Gamma1 with respect to q1
%                Jq12 is the partial der. of Gamma1 with respect to q2
%                Jq21 is the partial der. of Gamma2 with respect to q1
%etc...
Jq = [l1*sin(q1)+l2*sin(q1+phi1)              0;
      -l1*cos(q1)-l2*cos(q1+phi1)             0;
      0                           l4*sin(q2)+l3*sin(q2+phi2);
      0                           -l4*cos(q2)-l3*cos(q2+phi2)];


%In this matrix, Jphi11 is the partial der. of Gamma1 with respect to phi1
%                Jphi12 is the partial der. of Gamma1 with respect to phi2
%                Jphi21 is the partial der. of Gamma2 with respect to phi1
%etc...
Jphi = [l2*sin(q1+phi1)              0;
       -l2*cos(q1+phi1)             0;
      0                           l3*sin(q2+phi2);
      0                           -l3*cos(q2+phi2)];
  
  
  