%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  [T]= direct_kinematics_3RRR_numerical(ROBOT, THETA, threshold, max_iter)
%
%  Compute the direct kinematic problem in position for a 3RRR arm.
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
%  THETA: The input vector of active joint coordinates THETA = [theta1
%           theta2 theta3];
%  Threshold: Target sum of squares. When the sum of squares of the
%           function values is below this value the method stops (default 0.01).
%  Max_iter: Maximum number of iteration steps in the algorithm (default 50). 
%   
%  WARNING: The geometric parameters have been fixed in this function, they
%  should be extracted from the robot structure.
%
%  
% THE FUNCTION SHOWS HOW TO SOLVE THE DIRECT KINEMATICS OF THE 3RRR USING A
% NUMERICAL METHOD. THIS METHOD MAY BE APPLIED TO ANY PARALELL ROBOT FOR WHICH YOU
% MAY NOT FIND A CLOSED SOLUTION
% The solution uses a vanilla Newton-Raphson method, also known as a
% Levenberg-Marquardt method with Lambda=1.
% 
%   Please, for more information see:
%   http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
%   
% Example:
%   >>robot=load_robot('example','3RRR');
%   >>T=directkinematics_3RRR_numerical(robot, [pi/2 pi/4 -pi/2])
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
%   email: arturo.gil@umh.es date:   20/01/2014
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

function T=direct_kinematics_3RRR_numerical(robot, theta, threshold, max_iter)

global xQ yQ xR yR b1 b2 b3 a1 a2 a3 h

L=2.5; % basic mechanism length
%position of the fixed points xQ, yQ, xR, yR
xQ=L; yQ=0; xR=L/2; yR=L;

% Geometry
b1=1; b2=1; b3=1; a1=1; a2=1;a3=1; % m
%end effector is a triangle of side h
h=0.5; %m

%check convergence parameters
if ~exist('threshold','var')
    threshold=0.01;
end

if ~exist('max_iter','var')
    max_iter=50;
end


%input parameters for the direct kinematics
th1=theta(1);%1.9948;
th2=theta(2);%3.2869;
th3=theta(3);%-1.3998;

theta = [th1 th2 th3];

%initial beta, different solutions may be found, depending on the 
%starting values for the parameters beta
% beta = [xA yA Phi phi1 phi2 phi3]
beta=[0.5 0.5 0 0 0 0]';

%the vector y is the vector of values that the Gamma equations should have
%in this case, they should be all null
% Gamma=[Gamma1, Gamma2, Gamma3..., Gamma6];
y=[0 0 0 0 0 0]';

fs=[];

for i=1:max_iter,    
    f=compute_gamma_values(beta,theta);
    J=compute_jacobian_beta(beta,theta);
    
    delta=inv(J'*J)*J'*(y-f);
    
    %update beta
    beta = beta + delta;   
    
    sumofsqes=(y-f)'*(y-f);
    fs=[fs sumofsqes];
    
      if sumofsqes < threshold  
        fprintf('\ndirect_kinematics_3RRR::Solution found in %d iterations',i);
        fprintf('\nPlease note that only ONE possible solution is returned');
        break;
    end
end

%check whether the algorithm converged in less than max_iter steps
if i==max_iter
  fprintf('ERROR:: direct_kinematics_3RRR::Could not converge in %d iterations',i);
end

%build solution from the beta parameters.
T=eye(4);
T(1,4)=beta(1);
T(2,4)=beta(2);
T(1,1)=cos(beta(3));
T(2,1)=sin(beta(3));
T(1,2)=-sin(beta(3));
T(2,2)=cos(beta(3));


%draw the given solution
%q=[th1 phi1 th2 phi2 th3 phi3]
q=[th1 beta(4) th2 beta(5) th3 beta(6)];
drawrobot3d(robot, q), pause(2);

figure, plot(fs), title('Sum of squares vs. iteration step')


%compute the gamma functions, each function fi corresponds to a loop closing
%equation in the 3RRR mechanism, given the current estimates of beta
%the theta (theta1, theta2, theta3) may be considered here as constants,
%since they are known values for the direct kinematic solution
function f=compute_gamma_values(beta, theta)

global xQ yQ xR yR b1 b2 b3 a1 a2 a3 h

%just assign the values to some variables, so that the equations are easier
%to read
xA=beta(1);
yA=beta(2);
Phi=beta(3);
phi1=beta(4);
phi2=beta(5);
phi3=beta(6);

%known values of the direct kinematic problems
th1=theta(1);
th2=theta(2);
th3=theta(3);

%Loop closing equations for the 3RRR mechanism
f1=eval('xA-a1*cos(th1)-b1*cos(th1+phi1)');
f2=eval('yA-a1*sin(th1)-b1*sin(th1+phi1)');
f3=eval('xA-xQ-a2*cos(th2)-b2*cos(th2+phi2)+h*cos(Phi)');
f4=eval('yA-yQ-a2*sin(th2)-b2*sin(th2+phi2)+h*sin(Phi)');
f5=eval('xA-xR-a3*cos(th3)-b3*cos(th3+phi3)+h*cos(Phi+pi/3)');
f6=eval('yA-yR-a3*sin(th3)-b3*sin(th3+phi3)+h*sin(Phi+pi/3)');

%form vector with values,
%note that the f functions are the loop closing equations that should be 
%equal to zero
f=[f1 f2 f3 f4 f5 f6]';


%Compute Jacobian with respect to beta parameters
%xA yA Phi, ph1, ph2, ph3
function J=compute_jacobian_beta(beta, theta)

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
th1=theta(1);
th2=theta(2);
th3=theta(3);

%Compute Jacobian
%Gamma1 with respect to xA yA Phi, ph1, ph2, ph3
J11=1;
J12=0;
J13=0;
J14=b1*sin(th1+ph1);
J15=0;
J16=0;

%Gamma2 with respect to xA yA Phi, ph1, ph2, ph3
J21=0;
J22=1;
J23=0;
J24=-b1*cos(th1+ph1);
J25=0;
J26=0;
  
%Gamma3 with respect to xA yA Phi, ph1, ph2, ph3
J31=1;
J32=0;
J33=-h*sin(Phi);
J34=0;
J35=b2*sin(th2+ph2);
J36=0;

%Gamma4 with respect to xA yA Phi, ph1, ph2, ph3
J41=0;
J42=1;
J43=h*cos(Phi);
J44=0;
J45=-b2*cos(th2+ph2);
J46=0;

%Gamma5 with respect to xA yA Phi, ph1, ph2, ph3
J51=1;
J52=0;
J53=-h*sin(Phi+pi/3);
J54=0;
J55=0;
J56=b3*sin(th3+ph3);
  
%Gamma6 with respect to xA yA Phi, ph1, ph2, ph3
J61=0;
J62=1;
J63=h*cos(Phi+pi/3);
J64=0;
J65=0;
J66=-b3*cos(th3+ph3);

J=[J11 J12 J13 J14 J15 J16;
   J21 J22 J23 J24 J25 J26;
   J31 J32 J33 J34 J35 J36;
   J41 J42 J43 J44 J45 J46;
   J51 J52 J53 J54 J55 J56;
   J61 J62 J63 J64 J65 J66];
  



