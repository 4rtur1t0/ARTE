%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  [T]= direct_kinematics_3PRR_numerical(ROBOT, DISTANCE, threshold, max_iter)
%
%  Compute the direct kinematic problem in position for a 3PRR arm.
%  Returns the matrix T with position/orientation of the end effector.
%
%   T=[cos(Phi) -sin(Phi) 0 xA;
%      sin(Phi) cos(Phi) 0  yA;
%        0         0     1   0;
%        0          0    0   1];
%
%   The found solution of joint coordinates is drawn.
%
%  Input parameters:
%  ROBOT: robot structure (parallel).
%  DISTANCE: The input vector of active joint coordinates distance = [d1
%  d2 d3];
%  Threshold: Target sum of squares. When the sum of squares of the
%  function values is below this value the method stops (default 0.01).
%  Max_iter: Maximum number of iteration steps in the algorithm (default 50). 
%   
%  WARNING: The geometric parameters have been fixed in this function, they
%  should be extracted from the robot structure.
%
%  
% THE FUNCTION SHOWS HOW TO SOLVE THE DIRECT KINEMATICS OF THE 3PRR USING A
% NUMERICAL METHOD. THIS METHOD MAY BE APPLIED TO ANY PARALELL ROBOT FOR WHICH YOU
% MAY NOT FIND A CLOSED SOLUTION
% The solution uses a vanilla Newton-Raphson method, also known as a
% Levenberg-Marquardt method with Lambda=1.
% 
%   Please, for more information see:
%   http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
%   
% Example:
%   >>robot=load_robot('example','3PRR');
%   >>T=directkinematic(robot, [])
%
%   Authors: 
%           Rafael López Contreras
%           Francisco Martínez Femenía
%           Santiago Giménez García 
%           Albano López Gámez
%           Guillermo Salinas López
%           Jonatan Lloret Reina
%
%Universidad Miguel Hernandez de Elche. 
%   email: arturo.gil@umh.es date:   10/02/2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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

function T=direct_kinematics_3PRR_numerical(robot, d, threshold, max_iter)

global xQ yQ xR yR b1 b2 b3 a1 a2 a3 h

L=2.5; % basic mechanism length
%position of the fixed points xQ, yQ, xR, yR
xQ=1; yQ=0; xR=0; yR=1.84;

% Geometry
b1=1; b2=1; b3=1; %a1=1; a2=1;a3=1; % m
%the end effector is a triangle of side h
h=0.5; %m

%check convergence parameters
if ~exist('threshold','var')
    threshold=0.01;
end

if ~exist('max_iter','var')
    max_iter=50;
end


%input parameters for the direct kinematics
d1=d(1);%1.9948;
d2=d(2);%3.2869;
d3=d(3);%-1.3998;

distance = [d1 d2 d3];

%initial beta, different solutions may be found, depending on the 
%starting values for the parameters beta
% beta = [xA yA Phi Phi1 Phi2 Phi3]
T=eye(4);
T(1,4)=1.7;
T(2,4)=0.9;

Q = inversekinematic_3PRR(1, T);
    
beta=[1.7 0.9 0 Q(4) Q(5)   Q(6)]';


%the vector y is the vector of values that the fi equations should have
%in this case, they should be all null
y=[0 0 0 0 0 0]';

fs=[];

for i=1:max_iter,    
    f=compute_gamma_values(beta,distance);
    J=compute_jacobian_beta(beta,distance);
    
    delta=inv(J'*J)*J'*(y-f);
    
    %update beta
    beta = beta + delta;   
    
    sumofsqes=(y-f)'*(y-f);
    fs=[fs sumofsqes];
    
    if sumofsqes < threshold  
        fprintf('\ndirect_kinematics_3PRR::Solution found in %d iterations',i);
        fprintf('\nPlease note that only ONE possible solution is returned');
        break;
    end
end

%check whether the algorithm converged in less than max_iter steps
if i==max_iter
  fprintf('ERROR:: direct_kinematics_3PRR::Could not converge in %d iterations',i);
end

%build solution from the beta parameters.
T=eye(4);
T(1,4)=beta(1);
T(2,4)=beta(2);
T(1,1)=cos(beta(3));
T(2,1)=sin(beta(3));
T(1,2)=-sin(beta(3));
T(2,2)=cos(beta(3));


xA=beta(1);
yA=beta(2);
phi=beta(3);

%Position of the point B in base coordinates
xB=xA+h*cos(phi);
yB=yA+h*sin(phi);

%Position of the point C in base coordinates
xC=xA+h*cos(phi+pi/3);
yC=yA+h*sin(phi+pi/3);

%Dibujamos los eslabones y el triangulo del PRR

figure,
plot(0,0,'r.'), hold
plot(xA, yA,'k.')
plot(xB, yB,'k.')
plot(xC, yC,'k.')

plot_line([d1 0 0], [xA yA 0], 'm', 2)
plot_line([xQ+d2 0 0], [xB yB 0], 'g', 2)
plot_line([d3 yR 0], [xC yC 0], 'y', 2)

plot_line([0 0 0], [d1 0 0], 'm', 2)
plot_line([xQ 0 0], [xQ+d2 0 0], 'g', 2)
plot_line([0 yR 0], [d3 yR 0], 'y', 2)

plot_line([xA yA 0], [xB yB 0], 'r', 2)
plot_line([xB yB 0], [xC yC 0 0], 'r', 2)
plot_line([xC yC 0], [xA yA 0], 'r', 2)


figure, plot(fs), title('Sum of squares vs. iteration step')


%compute the gamma functions, each function fi corresponds to a loop closing
%equation in the 3PRR mechanism, given the current estimates of beta
%the theta (d1, d2, d3) may be considered here as constants,
%since they are known values for the direct kinematic solution
function f=compute_gamma_values(beta, d)

global xQ yQ xR yR b1 b2 b3 a1 a2 a3 h

%just assign the values to some variables, so that the equations are easier
%to read
xA=beta(1);
yA=beta(2);
Phi=beta(3);
phi1=beta(4);
phi2=beta(5);
phi3=beta(6);

%known values of the direct kinematic problem
d1=d(1);
d2=d(2);
d3=d(3);

%Loop closing equations for the 3RRR mechanism
f1=eval('xA-d1-b1*cos(phi1)');
f2=eval('yA-b1*sin(phi1)');
f3=eval('xA+h*cos(Phi)-xQ-d2-b2*cos(phi2)');
f4=eval('yA+h*sin(Phi)-yQ-b2*sin(phi2)');
f5=eval('xA+h*cos(Phi+pi/3)-xR-d3-b3*cos(phi3)');
f6=eval('yA+h*sin(Phi+pi/3)-yR-b3*sin(phi3)');

%form vector with values,
%note that the f functions are the loop closing equations that should be 
%equal to zero
f=[f1 f2 f3 f4 f5 f6]';


%Compute Jacobian with respect to beta parameters
%xA yA Phi, ph1, ph2, ph3
function J=compute_jacobian_beta(beta, d)

global xQ yQ xR yR b1 b2 b3 a1 a2 a3 h

%just assign the values to some variables, so that the equations are easier
%to read
%xA=beta(1);
%yA=beta(2);
Phi=beta(3);
ph1=beta(4);
ph2=beta(5);
ph3=beta(6);

%known values of the direct kinematic problems
d1=d(1);
d2=d(2);
d3=d(3);

%Compute Jacobian
%Gamma1 with respect to xA yA Phi, ph1, ph2, ph3
J11=1;
J12=0;
J13=0;
J14=b1*sin(ph1);
J15=0;
J16=0;

%Gamma2 with respect to xA yA Phi, ph1, ph2, ph3
J21=0;
J22=1;
J23=0;
J24=-b1*cos(ph1);
J25=0;
J26=0;
  
%Gamma3 with respect to xA yA Phi, ph1, ph2, ph3
J31=1;
J32=0;
J33=-h*sin(Phi);
J34=0;
J35=b2*sin(ph2);
J36=0;

%Gamma4 with respect to xA yA Phi, ph1, ph2, ph3
J41=0;
J42=1;
J43=h*cos(Phi);
J44=0;
J45=-b2*cos(ph2);
J46=0; 

%Gamma5 with respect to xA yA Phi, ph1, ph2, ph3
J51=1;
J52=0;
J53=-h*sin(Phi+pi/3);
J54=0;
J55=0;
J56=b3*sin(ph3);
  
%Gamma6 with respect to xA yA Phi, ph1, ph2, ph3
J61=0;
J62=1;
J63=h*cos(Phi+pi/3);
J64=0;
J65=0;
J66=-b3*cos(ph3);

J=[J11 J12 J13 J14 J15 J16;
   J21 J22 J23 J24 J25 J26;
   J31 J32 J33 J34 J35 J36;
   J41 J42 J43 J44 J45 J46;
   J51 J52 J53 J54 J55 J56;
   J61 J62 J63 J64 J65 J66];


function plot_line(p0, p1, color, w)
x0 = p0(1);
y0 = p0(2);
z0 = p0(3);
x1 = p1(1);
y1 = p1(2);
z1 = p1(3);
% Draw a line between p0 and p1
plot3([x0;x1],[y0;y1],[z0;z1], color, 'LineWidth',w);   

