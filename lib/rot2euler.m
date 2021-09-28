%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [alpha, beta, gamma] = rot2euler(R, convention)
%   Returns the Euler angles alpha, beta and gamma that yield a given matrix
%   R. The convention specifies the order and axes of rotations.
%   Currently, only the XYZ convention is supported. 
%
%   In particular, the function computes the angles alpha, beta and gamma that
%   allow to compute R as.
%
%   R = Rot(alpha,'x')*Rot(beta,'y')*Rot(gamma,'z')
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. email:
%   arturo.gil@umh.es date:   11/11/2020
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
function [sol1, sol2] = rot2euler(R, convention)


if convention=='XYZ'
    [sol1, sol2]=conventionXYZ(R);
elseif convention=='ZYX'
    [sol1, sol2]=conventionZYX(R);   
else
    'Unknown convention. Only XYZ and ZYX are supported'
end



function [sol1, sol2] = conventionXYZ(R)
%'R(1,3)=sen(beta)=1??'
if abs(R(1,3)) == 1
    % degenerate case in which sen(beta)=+-1 and cos(beta)=0
    alpha1 = 0; % arbitrarily set alpha to zero
    alpha2 = pi; % arbitrarily set alpha to pi
    beta1 = asin(R(1,3));
    beta2 = pi-beta1;
    if sin(beta1) > 0
        gamma1 = atan2(R(2,1), -R(3,1));   
        gamma2 = atan2(R(2,1), -R(3,1)) - alpha2;   
    else
        gamma1 = atan2(R(2,1), R(3,1));
        gamma2 = atan2(R(2,1), R(3,1))+alpha2; 
    end
else
    beta1 = asin(R(1,3));
    beta2 = pi-beta1;
    
    % standard way to compute alpha beta and gamma
    alpha1 = -atan2(R(2,3)/cos(beta1), R(3,3)/cos(beta1));
    alpha2 = -atan2(R(2,3)/cos(beta2), R(3,3)/cos(beta2));
    gamma1 = -atan2(R(1,2)/cos(beta1), R(1,1)/cos(beta1));
    gamma2 = -atan2(R(1,2)/cos(beta2), R(1,1)/cos(beta2));
end
% Normalize all angles to -pi, pi    
sol1 = [alpha1, beta1, gamma1];
sol2 = [alpha2, beta2, gamma2];
sol1 = normalize_angles(sol1);
sol2 = normalize_angles(sol2);


function [sol1, sol2] = conventionZYX(R)
%R(3,1)=sin(beta)=+-1?? --> degenerate case
% degenerate case in which sen(beta)=+-1 and cos(beta)=0
if abs(R(3,1)) == 1 
    alpha1 = 0; % arbitrarily set alpha to zero
    alpha2 = pi;%arbitrarily set to pi 
    beta1 = asin(-R(3,1));
    beta2 = pi-beta1;             
    if sin(beta1) > 0
        gamma1 = atan2(R(1,2), R(2,2));    
        gamma2 = atan2(R(1,2), R(2,2)) + alpha2;
    else
        gamma1 = atan2(-R(1,2), R(2,2));    
        gamma2 = atan2(-R(1,2), R(2,2)) - alpha2;
    end
else
    % standard way to compute alpha beta and gamma outside of the gimbal
    % lock
    beta1 = asin(-R(3,1));
    beta2 = pi-beta1;
    
    alpha1 = atan2(R(2,1)/cos(beta1), R(1,1)/cos(beta1));
    alpha2 = atan2(R(2,1)/cos(beta2), R(1,1)/cos(beta2));
    gamma1 = atan2(R(3,2)/cos(beta1), R(3,3)/cos(beta1));
    gamma2 = atan2(R(3,2)/cos(beta2), R(3,3)/cos(beta2));
end
% Normalize all angles to -pi, pi    
sol1 = [alpha1, beta1, gamma1];
sol2 = [alpha2, beta2, gamma2];
sol1 = normalize_angles(sol1);
sol2 = normalize_angles(sol2);

function sol_norm = normalize_angles(sol)
sol_norm = [0 0 0];
for i=1:3
    sol_norm(i) = atan2(sin(sol(i)), cos(sol(i)));
end
    

