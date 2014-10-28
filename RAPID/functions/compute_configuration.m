%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONF= COMPUTE_CONFIGURATION(Q) 
% Returns the configuration corresponding to the specified joint coordinates
%
% In RAPID the configuration variable is used to distinguish among the
% different solutions for the inverse kinematic problem. 
% The variable conf=[cf1 cf4 cf6 cfx] is formed by the position of the
% first, fourth and sixth robot axis. 
%
% If the angle is positive, the conf value has one of the
% following values:
%         1 | 0
%        ---|---
%         2 | 3
% that can be computed with the formulae: floor(joint_angle x 2/pi)
% if the angle is negative, then
%         -3 | -4
%         ---|---
%         -2 | -1
% computed as: ceil(joint_angle × 2/pi - 1)
%
%  The last value cfx is necessary for robots that possess eight possible
%  solutions for the inverse kinematic problem in position/orientation.
%  Otherwise, specifying only three angles with cf1 cf4 and cf6 would not suffice.
%  
%  This is the case of the IRB140, the IRB 6650, IRB120, IRB52... and more.
%  Other robots manufactured by others may have similar ways to define a
%  particular solution of the inverse kinematic.
%
%  For a complete description, see:
%  http://developercenter.robotstudio.com/BlobProxy/manuals/RapidIFDTechRefManual/doc439.html
%
% The compute configuration function gives different values depending on the robots name
% 
% The compute_configuration function calls different internal functions to
% give the correct configurations values, depending on the robot model. The
% last value cfx is not computed and rather given. This value depends
% directly on the inversekinematic function defined for any particular
% function. The inversekinematic function returns the different solutions
% in a given specified order
%
%   
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
function conf=compute_configuration(robot, q)
%initialize conf data
conf = zeros(1,4);

switch robot.name
    case 'ABB_IRB140_M2000'
        conf=compute_configuration_standard_6R(robot, q);
    case 'ABB_IRB52' 
        conf=compute_configuration_standard_6R(robot, q);
    case 'ABB_IRB120'
        conf=compute_configuration_standard_6R(robot, q);
        
        %if the robot is not found before, compute it as the standard way
    otherwise 
        disp('WARNING: the compute_configuration function did not find a suitable method to compute the configuration');
        disp('WARNING: using a standard function to compute it');
        conf=compute_configuration_standard_6R(robot, q);
end



% Standard way of computing configuration for robots with 6 rotational
% joints. Examples include:
% IRB140, the IRB 6650, IRB120, IRB52
% The cfx is computed as described in:
%
%  http://developercenter.robotstudio.com/BlobProxy/manuals/RapidIFDTechRefManual/doc439.html
% cfx | Wrist center relative to axis 1 | Wrist center relative to lower arm | Axis 5 angle
% 0   | In front of                     | In front of                        | Positive	
% 1   | In front of                     | In front of                        | Negative	
% 2   | In front of                     | Behind                             | Positive	
% 3   | In front of                     | Behind                             | Negative	
% 4   | Behind                          | In front of                        | Positive	
% 5   | Behind                          | In front of                        | Negative	
% 6   | Behind                          | Behind                             | Positive	
% 7   | Behind                          | Behind                             | Negative	
%
% The table is implemented with three values a, b and c, with the following
% values:
% table_values=[a b c];
% In front of a=0, Behind   a=1
% In front of b=0, Behind   b=1
% Positive    c=0, Negative c=1
function conf=compute_configuration_standard_6R(robot, q)

if q(1)>=0
    cf1 = floor(q(1)*2/pi);
else
    cf1 = ceil(q(1)*2/pi - 1);
end
%temp value for conf
conf = [cf1 0 0 0];

if robot.DOF<4
   return; 
end

if q(4)>=0
    cf4 = floor(q(4)*2/pi);
else
    cf4 = ceil(q(4)*2/pi) - 1;
end
%temp value for conf
conf = [cf1 cf4 0 0];

if robot.DOF<6
   return; 
end

if q(6)>=0
    cf6 = floor(q(6)*2/pi);
else
    cf6 = ceil(q(6)*2/pi) - 1;
end


T=directkinematic(robot, q);

table_values=[];

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);
T12=dh(robot, q, 2);
T23=dh(robot, q, 3);
T34=dh(robot, q, 4);
T45=dh(robot, q, 5);

T05=T01*T12*T23*T34*T45;
%compute the wrist point Pw for the robot, given the current joint values q
Pw=T05(1:3,4);

% Express Pw in the reference system 1, for convenience
%pa = inv(T01)*[Pw; 1];
pa = (T01)\[Pw; 1];

% The case indicated with Wrist center relative to axis 1 "In front of"
% happens when the X coordinate of pa is positive (value=0). Otherwise, it is 1 and
% indicated as Behind.
if pa(1)>=0
    table_values(1)=0;
else
    table_values(1)=1;
end

% Express Pw in the reference system 2, for convenience
%pb = inv(T01*T12)*[Pw; 1];
pb = (T01*T12)\[Pw; 1];

% The case indicated with Wrist center relative to lower arm "In front of"
% happens when the Y coordinate of pb is positive (value=0). Otherwise, it is 1 and
% indicated as Behind.
if pb(2)>=0
    table_values(2)=0;
else
    table_values(2)=1;
end


%finally, the last value refers to the sign of q(5), positive=0
%negative=1
if q(5)>=0
    table_values(3)=0;
else
    table_values(3)=1;
end


%now give the cfx value according to the specified table
if isequal(table_values, [0 0 0])
     cfx=0;
elseif isequal(table_values, [0 0 1])
     cfx=1;
elseif isequal(table_values, [0 1 0])
     cfx=2;
elseif isequal(table_values, [0 1 1])
     cfx=3;
elseif isequal(table_values, [1 0 0])
     cfx=4;
elseif isequal(table_values, [1 0 1])
     cfx=5;
elseif isequal(table_values, [1 1 0])
     cfx=6;
else
     cfx=7;
end

conf = [cf1 cf4 cf6 cfx];

