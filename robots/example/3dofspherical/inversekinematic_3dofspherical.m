%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the 3dof spherical robot
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions for q = [theta beta delta] that place the end effectors at the
% position specified by T. Two possible solutions are returned,
% generally called elbow in and elbow out:
%  
% FIRST column contain elbow OUT solution
% SECOND column contains elbow IN solution
% 
%   Author: Ángel Rodríguez 
%   email: arodgre@gmail.com date:   18/12/2013
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
function q = inversekinematic_3dofspherical(robot, T)

fprintf('\nComputing inverse kinematics for the %s robot.\n', robot.name);

L1=robot.L1;
L2=robot.L2;

%Eprima is the robot's arm end
Eprima=T(1:3,4);
if Eprima(3)==0
    L2prima=L2;
    delta=0;
else
    delta=asin(Eprima(3)/L2);%Angle between arm 2 and the projection
    L2prima= Eprima(3)/tan(delta);
end

%Distance of the projection on the XYplane to the origin. 
Ro=sqrt(Eprima(1)^2+Eprima(2)^2);
%Distance of the point to the origin. 
Xi=sqrt(Eprima(1)^2+Eprima(2)^2+Eprima(3)^2);
%Passive joint's angle. Between arm1 & arm2
betaprima= real(acos((L2^2+L1^2-Xi^2)/(2*L2*L1)));
beta= pi - betaprima;


phi= atan2(Eprima(2),Eprima(1));
gamma= real(acos((L1^2+Ro^2-L2prima^2)/(2*Ro*L1)));
theta=[phi-gamma, phi+gamma];%phi-gamma is the out elbow solution
                               %phi+gamma is the in elbow solution


%q contains elbow in and elbow out solution
%FIRST column contain elbow OUT solution
%SECOND column contains elbow IN solution
q=[theta(1), theta(2);
       beta,    -beta;
      delta,    delta];

%Puesto en cuarentena porque hay muchas opciones de brazos.
% if q(2)<pi & q(2)>-0.35
%     fprintf('\nWARNING: inversekinematic_3dofplanar: unfeasible solution. The arms impact aganinst each other.\n');
%     %set(gca,'color','r');
% end 

if Xi > (L1+L2)
   fprintf('WARNING: inversnekinematic_3dofplanar: unfeasible solution. The point cannot be reached.\n'); 
   %set(gca,'color','r');
end


