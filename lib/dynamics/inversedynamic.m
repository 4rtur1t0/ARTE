%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  TAU = INVERSEDYNAMIC(Q, QD, QDD, GRAV, FEXT)
%  TAU= INVERSEDYNAMIC: Compute inverse dynamics via recursive Newton-Euler
%  formulation. 
%  TAU = INVERSEDYNAMIC(Q, QD, QDD, GRAV, FEXT) computes the
%  required torques (Nm) and forces (N) required to instantaneously bring
%  the arm to the specified state given by positions Q, speed QD,
%  acceleration QDD. The gravity is expressed in the base coordinate
%  system, typically defined as GRAV = [0 0 9.81]'. In addition, FEXT
%  is a 6-vector [Fx Fy Fz Mx My Mz] defining the forces and moments
%  externally applied to the end-effector (in coordinates of the end effector's n-th DH
%  system).
%
%   Example of use:
%   q=[0 0 0 0 0 0]
%   qd=[0 0 0 0 0 0]
%   qdd = [0 0 0 0 0 0]
%   g=[0 0 9.81]';
%   fext = [0 0 0 0 0 0]';
%   puma560=load_robot('puma', '560');
%	tau = inversedynamic(puma560, q, qd, qdd, g, fext)
%   where: q is the position of the arm, qd the joint velocities and qdd
%   the accelerations. The vector g defines the gravity in the base reference 
%   system, whereas the vector fext = [fx fy fz Mx My Mz] defines the
%   forces and moments acting on the end effector's reference system.
%
%   Author: Arturo Gil Aparicio, arturo.gil@umh.es
%   
%   Bibliography: The algorithm has been implemented as is "ROBOT ANALYSIS. The mechanics of Serial and Parallel
%        manipulators". Lung Weng Tsai. John Wiley and Sons, inc. ISBN:
%        0-471-32593-7. pages: 386--391.
%            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function tau = inversedynamic(robot, q, qd, qdd, grav, fext)
tau = zeros(robot.DOF, 1);

if robot.debug
    fprintf('\nComputing inverse dinamics for the %s robot', robot.name);
end

%if the robot has not dynamic parameters, exit 
if ~robot.has_dynamics
    fprintf('\nRobot %s has not dynamic parameters set. Exiting', robot.name);
    return;
end

%Each Z vector expressed in its own reference frame, just to make equations
%readable
z0 = [0;0;1];

%evaluate robot parameter DH table for current variable q
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);
n=length(theta);

m = robot.dynamics.masses; % column vector with the mass of every link

% position of the COM of each link, one row per link
% the position of the COM of each link is expressed with respect to
% its own reference system
r_com = robot.dynamics.r_com; 

%initial values for
vdi = zeros(3,1); %linear acceleration, expressed in the ith frame
wi = zeros(3,1);  %angular speed, expressed in the ith frame
wdi = zeros(3,1); %angular acceleration, expressed in the ith frame
%vi = zeros(3,1); %linear velocity, expressed in the ith frame

%Store forces and moments at the COM of each link
F_com = zeros(3,n);
N_com = zeros(3,n);

%First compute motion forward from robot base
% to the last link
for j=1:n,
    A=dh(theta(j), d(j), a(j), alfa(j));
    %Note: compute the inverse by transpose.
    %this implies a transformation from the {i-1}th system to the ith
    %system.
    R = A(1:3,1:3)'; 
    %position vector of the origin of the ith link frame with respect to
    %the (i-i)th link frame
    ri = [a(j); d(j)*sin(alfa(j)); d(j)*cos(alfa(j))];
    
    if robot.kind(j) == 'R', %rotational axis
        %Note, should be computed in the specified order.
        wdi = R*(wdi + z0*qdd(j) + cross(wi,z0*qd(j)));
        wi = R*(wi + z0*qd(j));
        vdi = R*vdi + cross(wdi,ri) + cross(wi, cross(wi,ri));
    else% prismatic axis
        wdi = R*wdi;
        wi = R*wi;        
        vdi = R*(vdi + z0*qdd(j)) + cross(wdi,ri) + cross(wi, cross(wi,ri)) + 2*cross(wi,R*z0*qd(j));
    end
    
    %Inertia matrix
    J = [robot.dynamics.Inertia(j,1) robot.dynamics.Inertia(j,4) robot.dynamics.Inertia(j,6);
        robot.dynamics.Inertia(j,4) robot.dynamics.Inertia(j,2) robot.dynamics.Inertia(j,5);
        robot.dynamics.Inertia(j,6) robot.dynamics.Inertia(j,5) robot.dynamics.Inertia(j,3)	];
    %acceleration of the center of mass of link i
    v_comi = vdi + cross(wdi,r_com(j,:)') + cross(wi,cross(wi,r_com(j,:)'));
    % Force as Newton's law F=m*a of COM
    F = m(j)*v_comi;
    N = J*wdi + cross(wi,J*wi);
    %Finally, store the force and moment to use it in the backward's
    %computation loop, next
    F_com(:,j) = F;
    N_com(:,j) = N;
end

%assure fext is a column vector
fext=fext(:);
%  backward computations
ifi = fext(1:3);		% forces expressed in the last reference system
ini = fext(4:6);     % moments at the last reference system.

T=directkinematic(robot, q);
%vector g, expressed in the last reference system
gj=T(1:3,1:3)\grav;

for j=n:-1:1,
    ri = [a(j); d(j)*sin(alfa(j)); d(j)*cos(alfa(j))];
    
    %compute resultant force on the i-1 (minus) link
    ifim = F_com(:,j) - m(j)*gj + ifi;
    
    %compute resultant moment on i-1
    inim = N_com(:,j) + cross(ri+r_com(j,:)',ifim) - cross(r_com(j,:)', ifi) + ini ;
    
    
    %Transform the resultant forces and moments to the previous reference system, 
    %so that the recursive equations are always to be added on the
    %same reference system.
    A=dh(theta(j), d(j), a(j), alfa(j));
    R=A(1:3,1:3);
    ifi=R*ifim;
    ini=R*inim;
    gj = R*gj; % yes, now, change gj to the previous reference system
    
    %Now, project the moments to the z0 axes 
    if robot.kind(j) == 'R', %rotational joint      
        tau(j) = ini'*z0;% + robot.motors.G(j)^2*robot.motors.Inertia(j)*qdd(j);
    else % prismatic joint        
        tau(j) = ifi'*z0;% + robot.motors.G(j)^2*robot.motors.Inertia(j)*qdd(j);
    end
    %add viscous friction in this case
    if robot.dynamics.friction
        %Note: we account for two types of friction: viscous and coulomb
        tau(j) = tau(j) + friction(robot, qd, j);
    end
end 