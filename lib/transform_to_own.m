%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Script used to change the coordinates of the points stored in STL format
%   The coordinates in files link0_base.stl, link1_base.stl... are referred to
%   the base reference system. The script creates the files link0.stl,
%   link1.stl, link2.stl. The points in these files are referred to its own
%   D-H reference
%   USAGE: Please follow the steps below.
%   STEPS:
%       STEP 1: Name each of the links as link0_base.stl, link1_base.stl,
%       link2_base.stl. And save them under the same directory of your robot,
%       namely, robots/abb/IRB6620.
%       The vertices in each of the STL files are referred to the reference
%       system of the robot base. They are normally expressed in millimeters.
%       script 
%
%       STEP 2: In robots/abb/IRB6620/parameters.m, set robot.graphical.has_graphics=0
%
%       STEP 3: Now you should visualize the STL files. They are normally
%       referred to the robots base reference system.
%
%       STEP 4: Write the DH parameters of your robot.
%
%       STEP 5: The units of your STL files are generally millimeters. If they
%       are meters, change the variable mm2m=1 in the following call script.
%       Run transform_to_own
%
%       >>transform_to_own('ABB', 'IRB6620', 1000)
%
%       which changes the units in the stl file to the standard meters.
%       The script should create the files link0.stl, link1.stl, ... etc.
%       Each referred to its own reference system.
%
%       STEP 6: In robots/abb/IRB6620/parameters.m, set
%       robot.graphical.has_graphics=1. Now you've got graphics for your robot!
%
%       STEP 7: Load the robot:
%       >> robot=load_robot('abb', 'IRB6620');
%       >> drawrobot3d(robot, [0 0 0 0 0 0])
%       you should see your robot in 3D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

function transform_to_own(manufacturer,model, mm2m)

%save current path
path=pwd;

%load basic parameters for the robot
%CHANGE ROBOT name accordingly
robot = load_robot(manufacturer, model);

q=[0 0 0 0 0 0];
% Denavit-Hartenberg parameters of the robot
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);

n=length(theta); %# number of DOFs
%T=eye(4);
T=robot.T0;
for i=0:robot.DOF,
    fprintf('\nReading link %d BASE\n %s\n', i, sprintf([robot.path '/link%d_base.stl'], i));
    %read file in base reference system. Please note that link0 is already
    %defined in the base reference system. In the first loop T= identity
    [fout, vout, cout] = stl_read(sprintf([robot.path '/link%d_base.stl'], i));
    
    %change points from mm to meters.
    V=vout/mm2m;
    V(:,4) = ones(length(V),1); %homogeneous coordinates
   
    %You can also specify any homogeneous transformation matrix here
    %      T=[1 0 0 -0.06/2;
    %         0 1 0 0.1/2;
    %         0 0 1 0.02/2;
    %         0 0 0 1]
   
   %Or even re-scale the points
    %        T=[2 0 0 0;
    %          0 1/2 0 0;
    %          0 0 1/5 0;
    %          0 0 0 1];
    
    %inverse transform, the points V in link i are now referred to its own D-H reference system
    V = (inv(T)*V')';
    V  = V(:,1:3);
    fprintf('\nWriting link %d\n %s\n', i, sprintf([robot.path '/link%d.stl'], i));
    stlwrite(sprintf([robot.path '/link%d.stl'], i), fout, V, 'mode', 'ascii');
    %compute transformation for the next link
    if (i+1) > robot.DOF
        continue;
    end
    T = T*dh(theta(i+1), d(i+1), a(i+1), alfa(i+1));
end

%restore path
cd(path);

%load the robot again for changes to take place
robot = load_robot(manufacturer, model);

