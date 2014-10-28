%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse dynamics for the 2dof planar robot
%
%   tau = inversedynamics_2dofplanar_fast(robot, q, qd, qdd, fe)
%   
%   Where robot stores the kinematic and dynamic parameters for this robot.
%   q: joint positions.
%   qd: joint velocities.
%   qdd: joint accelerations.
%   fe: vector of external forces. (unused)
%
%   This function just executes the inverse dynamic model for this robot.
%   The equations to compute this dynamic model can be found in:
%   "ROBOT ANALYSIS. The mechanics of Serial and Parallel
%        manipulators". Lung Weng Tsai. John Wiley and Sons, inc. ISBN:
%        0-471-32593-7. page 405.
%
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 08/03/2014
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
function tau = inversedynamics_2dofplanar_fast(robot, q, qd, qdd, fe)

fprintf('\nComputing inverse dynamics for the %s robot', robot.name);

a = eval(robot.DH.a);
a1=a(1);
a2=a(2);

gc=9.81;

m1=robot.dynamics.masses(1);
m2=robot.dynamics.masses(2);

%express it as a general dynamic function
%M*qdd + V + G = Q, where Q is a vector of generalized forces or moments
% in this case, it is done just to represent the equations more clearly.
% have a look at the directdynamics_2dofplanar_fast function to observe
% this expression used to compute the direct dynamic model

%M is a 2x2 manipulator inertia matrix
M = [(m1*(1/3) + m2)*a1^2 + m2*a1*a2*cos(q(2)) + (1/3)*m2*a2^2 ... 
      (1/2)*m2*a1*a2*cos(q(2))+(1/3)*m2*a2^2 ;
      (1/2)*m2*a1*a2*cos(q(2))+(1/3)*m2*a2^2  (1/3)*m2*a2^2];

V=[-m2*a1*a2*sin(q(2))*(qd(1)*qd(2)+(1/2)*qd(2)^2);
    (1/2)*m2*a1*a2*sin(q(2))*qd(1)^2];

G=[gc*(((1/2)*m1+m2)*a1*cos(q(1)) + (1/2)*m2*a2*cos(q(1)+q(2)));
    (1/2)*m2*gc*a2*cos(q(1)+q(2))];
  

tau = M*qdd + V + G;


%A different way to write the equations.
% tau1=((m1*(1/3) + m2)*a1^2 + m2*a1*a2*cos(q(2)) + (1/3)*m2*a2^2)*qdd(1)+...
%     ((1/2)*m2*a1*a2*cos(q(2))+(1/3)*m2*a2^2)*qdd(2)...
% -m2*a1*a2*sin(q(2))*(qd(1)*qd(2)+(1/2)*qd(2)^2)+...
%     gc*(((1/2)*m1+m2)*a1*cos(q(1))+(1/2)*m2*a2*cos(q(1)+q(2)));
% 
% tau2=((1/2)*m2*a1*a2*cos(q(2))+(1/3)*m2*a2^2)*qdd(1)+(1/3)*m2*a2^2*qdd(2)+...
%     (1/2)*m2*a1*a2*sin(q(2))*qd(1)^2+(1/2)*m2*gc*a2*cos(q(1)+q(2));
% 
% tau = [tau1 tau2]';


% %Compute speeds
% 
% %Link 1
% w11=[0 0 qd(1)]';
% wd11=[0 0 qdd(1)]';
% vd1=a1*[-qd(1)^2 qdd(1) 0];
% vd1c=(a1/2)*[-qd(1)^2 qdd(1) 0];
% 
% %Link 2
% w22=[0 0 qd(1)+qd(2)]';
% wd22=[0 0 qdd(1)+qdd(2)]';
% vd2=[a1*(qdd(1)*sin(q(2))-qd(1)^2*cos(q(2)))-a2*(qd(1)+qd(2))^2;
%     a1*(qdd(1)*cos(q(2))+qd(1)^2*sin(q(2)))+a2*(qdd(1)+qdd(2));
%     0];
% vd2c=[a1*(qdd(1)*sin(q(2))-qd(1)^2*cos(q(2)))-(a2/2)*(qd(1)+qd(2))^2;
%     a1*(qdd(1)*cos(q(2))+qd(1)^2*sin(q(2)))+(a2/2)*(qdd(1)+qdd(2));
%     0];
% 
% g1=[-gc*sin(q(1)) -gc*cos(q(1)) 0]';
% g2=[-gc*sin(q(1)+q(2)) -gc*cos(q(1)+q(2)) 0]';
% 
% 
% f22a=-m2*vd2c;
% n22a=-(1/12)*m2*a2^2*wd22;
% f221=-f22a-m2*g2;

%f110=m2*()
