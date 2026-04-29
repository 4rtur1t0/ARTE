%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EXERCISE: WRITE THE LAGRANGE FORMULATION FOR A 6DOF ROBOT
% ROBOT with RRRRRR joints
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
function compute_dynamics_symbolic_lagrange_IRB140
%
% TODO
% Put rcom as a function of L1, L2, L3 and solve the error
% compute 0Ri to align I1, I2, I3 with wi
% compute G
syms q1 q2 q3 q4 q5 q6 real
syms qd1 qd2 qd3 qd4 qd5 qd6 real
syms m1 m2 m3 m4 m5 m6 real
syms L1 L2 L3 L4 L5 L6
syms I1x I1y I1z real
syms I2x I2y I2z real
syms I3x I3y I3z real
syms I4x I4y I4z real
syms I5x I5y I5z real
syms I6x I6y I6z real

q = [q1 q2 q3 q4 q5 q6];
qd = [qd1 qd2 qd3 qd4 qd5 qd6];
n=6;
robot.DOF=6;

% L1 = 0.352
% L2 = 0.07
% L3 = 0.36
% L4 = 0.38
% L5 = 0.065
robot.DH.theta=eval( '[q1   q2-pi/2   q3   q4   q5   q6+pi]');
robot.DH.d=     eval('[L1     0       0    L4    0    L5]');
robot.DH.a=    eval( '[L2     L3      0    0     0    0]');
robot.DH.alpha= eval('[-pi/2  0    -pi/2 pi/2 -pi/2  0]');

% Caution, the location of the COM is chosen as to 
% match the COM of the IRB140 robot in Coppelia Sim, not in the
% real robot
robot.dynamics.r_com=[ -L2	      L1/3    0;  %(rx, ry, rz) link 1
                      -L3/2	       0	       0;  %(rx, ry, rz) link 2
                        0	       0	       L4/8;
                        0         -L5          0;
                        0          0           0;
                        0          0           0]; %(rx, ry, rz) link 3
% Three Euler XYZ angles that relate the
% 0Ri orientation (DH) with the orientation of the COM.
robot.dynamics.euler_com=[ pi/2   0          0;      %(rx, ry, rz) link 1
                            0	   pi/2	    pi/2;  %(rx, ry, rz) link 2
                            0	   -pi/2	 pi;
                            0       pi/2    pi/2;
                            pi       0       0;
                            pi       0     -pi/2]; %(rx, ry, rz) link 3
                        
robot.dynamics.masses = [30 30 30 20 10 5];
                        
                        % Moments of inertia with respect to the COM of each link
%robot.dynamics.I=[  0.02146      0.02146    0.00125;  %(Ixx, Iyy, Izz) link 1
%                    0.02167	     0.02167    0.00167;  %(Ixx, Iyy, Izz) link 2
%                    0.02167	     0.02167    0.00167]; %(Ixx, Iyy, Izz) link 3
                
robot.dynamics.I=[I1x      I1y    I1z;  %(Ixx, Iyy, Izz) link 1
                  I2x      I2y    I2z;  %(Ixx, Iyy, Izz) link 2
                  I3x      I3y    I3z;
                  I4x      I4y    I4z;
                  I5x      I5y    I5z;
                  I6x      I6y    I6z];
                 

[M, C, G] = compute_inverse_dynamic_model(robot);
matlabFunction(M, "File", "func_M.m")
matlabFunction(C, "File", "func_C.m")
matlabFunction(G, "File", "func_G.m")




function [M, C, G] = compute_inverse_dynamic_model(robot)
disp('COMPUTING M')
M = compute_M(robot);
disp('COMPUTING C')
C = compute_C(M, robot.DOF);
disp('COMPUTING G')
G = compute_G(robot);

% Compute the manipulator inertia matrix.
function [M] = compute_M(robot)
syms q1 q2 q3 q4 q5 q6 real
syms qd1 qd2 qd3 qd4 qd5 qd6 real
syms m1 m2 m3 m4 m5 m6 real
syms L1 L2 L3 L4 L5 L6 real
syms I1x I1y I1z real
syms I2x I2y I2z real
syms I3x I3y I3z real
syms I4x I4y I4z real
syms I5x I5y I5z real
syms I6x I6y I6z real
syms g real
% the Jacobian of the center of mass
% the firs Jvc1 is null (null linear speed)
[Jvc1, Jw1] = compute_jacobians_linkj(robot, 1);
[Jvc2, Jw2] = compute_jacobians_linkj(robot, 2);
[Jvc3, Jw3] = compute_jacobians_linkj(robot, 3);
[Jvc4, Jw4] = compute_jacobians_linkj(robot, 4);
[Jvc5, Jw5] = compute_jacobians_linkj(robot, 5);
[Jvc6, Jw6] = compute_jacobians_linkj(robot, 6);

% global rotation matrices of the COM of link j
[r, R01_com] = compute_rcom_Rcom_linkj(robot, 1);
[r, R02_com] = compute_rcom_Rcom_linkj(robot, 2);
[r, R03_com] = compute_rcom_Rcom_linkj(robot, 3);
[r, R04_com] = compute_rcom_Rcom_linkj(robot, 4);
[r, R05_com] = compute_rcom_Rcom_linkj(robot, 5);
[r, R06_com] = compute_rcom_Rcom_linkj(robot, 6);

Ig1 = compute_global_I(R01_com, diag(robot.dynamics.I(1,:)));
Ig2 = compute_global_I(R02_com, diag(robot.dynamics.I(2,:)));
Ig3 = compute_global_I(R03_com, diag(robot.dynamics.I(3,:)));
Ig4 = compute_global_I(R04_com, diag(robot.dynamics.I(4,:)));
Ig5 = compute_global_I(R05_com, diag(robot.dynamics.I(5,:)));
Ig6 = compute_global_I(R06_com, diag(robot.dynamics.I(6,:)));

% compute M
M = m1*(Jvc1'*Jvc1) + m2*(Jvc2'*Jvc2) + m3*(Jvc3'*Jvc3); 
M = M + m4*(Jvc4'*Jvc4) + m5*(Jvc5'*Jvc5) + m6*(Jvc6'*Jvc6); 
M = M + Jw1'*Ig1*Jw1 + Jw2'*Ig2*Jw2 + Jw3'*Ig3*Jw3;
M = M + Jw4'*Ig4*Jw4 + Jw5'*Ig5*Jw5 + Jw6'*Ig6*Jw6;
disp('SIMPLIFY M')
M = simplify(M);


function [C] = compute_C(M, n)
syms q1 q2 q3 q4 q5 q6 real
syms qd1 qd2 qd3 qd4 qd5 qd6 real
q = [q1 q2 q3 q4 q5 q6];
qd = [qd1 qd2 qd3 qd4 qd5 qd6];
%n=6;
C = [];
% now compute V, the Coriollis term.
% compute Vi
for i=1:n
    disp('Computing Ci')
    i
    Ci = 0;
    for j=1:n
        for k=1:n
            fprintf('\n%d, %d, %d', i, j, k);
            a = diff(M(i, j), q(k));
            b = -(0.5)*diff(M(j, k), q(i));            
            %(a+b)*qd(j)*qd(k)
            Ci = Ci + (a+b)*qd(j)*qd(k);
        end
    end
    %Ci
    disp('Simplify Ci')
    Ci = simplify(Ci);
    C = [C; Ci];
end


function [G] = compute_G(robot)
% finally, compute G
% to ease the computation, we first define the total potential
% energy as P and then compute G1 as G1 = \part P/\part q1, 
% G2 = \part P/\partial q2 ... etc
syms q1 q2 q3 q4 q5 q6 real
syms m1 m2 m3 m4 m5 m6 real
syms L1 L2 L3 L4 L5 L6 real
syms g real

q = [q1 q2 q3 q4 q5 q6];
m = [m1 m2 m3 m4 m5 m6];
gv = [0 0 g]';
n = robot.DOF;
P = 0;
% compute the total potential energy of the masses
% of the manipulator
for i=1:n
   [rci, R] = compute_rcom_Rcom_linkj(robot, i);
   Pi = m(i)*gv'*rci;
   P = P + Pi;   
end

G = [];
for i=1:n
    Gi=diff(P, q(i));
    Gi = simplify(Gi);
    G = [G; Gi];
end


function [ocj, R0j_com] = compute_rcom_Rcom_linkj(robot, linkj)
% compute the position of the COM of linkj
A = eye(4);
for i=1:linkj
    A=A*dh_sym(robot.DH.theta(i), robot.DH.d(i), robot.DH.a(i), robot.DH.alpha(i));
end
% compute the relative transformation from link i to COM.
Ajcom = sym(eye(4));
Ajcom(1:3, 4) = robot.dynamics.r_com(linkj, :)';
% the center of masses of link j
ocj = A*Ajcom;
ocj = ocj(1:3, 4);

% comppute the rotation matrix of the reference system
% centered the COM
% rotation matrix of the reference system attached to the
% j-th link
R0j = A(1:3, 1:3);
% compute the relative transformation from link i to COM.
Rjcom = euler2rot(robot.dynamics.euler_com(linkj, :), 'XYZ');
Rjcom(abs(Rjcom) < 0.01)=0;
Rjcom = sym(Rjcom);
% finally compute the total transformation
R0j_com = R0j*Rjcom;


function [Jvcj, Jwj] = compute_jacobians_linkj(robot, linkj)
% compute the jacobian in linear speed and in angular
% speed for link j.
% find oci (oc1), oc2, oc3, the center of masses
% of link i, use the DH system i and the relative 
% vector r_com
[ocj, R] = compute_rcom_Rcom_linkj(robot, linkj);

T = eye(4);
Jvcj = [];
Jwj = [];
for i=1:linkj
    zi = T(1:3, 3);
    oi = T(1:3, 4);
    % caution: change if prismatic joint.
    Jvcj = [Jvcj cross(zi, ocj-oi)];
    Jwj = [Jwj zi];
    % compute next oci
    T = T*dh_sym(robot.DH.theta(i), robot.DH.d(i), robot.DH.a(i), robot.DH.alpha(i));    
end
% n DOFs
% complete the Jacobian with zeros
for i=(linkj+1):robot.DOF
    Jvcj = [Jvcj [0 0 0]'];
    Jwj = [Jwj [0 0 0]'];
end


function [Ig] = compute_global_I(R, I)
Ig = R*I*R';

function A = dh_sym(theta, d, a, alpha)
syms q1 q2 q3 q4 q5 q6
% avoid almost zero elements in cos(alpha) and sin(alpha)
ca = cos(alpha);
sa = sin(alpha);
if abs(ca) < 1e-6
    ca = 0;
end
if abs(sa) < 1e-6
    sa = 0;
end

A=[cos(theta)  -ca*sin(theta)   sa*sin(theta)   a*cos(theta);
   sin(theta)   ca*cos(theta)  -sa*cos(theta)   a*sin(theta);
            0         sa             ca             d;
            0         0               0             1];
        

        




