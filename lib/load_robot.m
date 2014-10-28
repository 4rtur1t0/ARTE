%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   LOAD_ROBOT Loads a data structure corresponding to the specified robot.
%
%
% 	robot = LOAD_ROBOT(MANUFACTURER, VERSION) returns a robot data
% 	structure of the specified robot.  Each robot parameters' are stored in
% 	the directory named robots/manufacturer/version. For example:  
%
%   Example: 
%   robot =  load_robot('ABB', 'IRB140')
%
%   gets the available data for the ABB IRB140 robotic arm whose
%   parameters are stored in the directory robots/ABB/IRB140
%   
%   robot = LOAD_ROBOT(PATH) specifies a relative path to the robots
%   directory. For example:
%   robot =  load_robot('ABB/IRB140')
%
%   robot = LOAD_ROBOT() with no arguments opens a dialog box to select a
%   robot's parameters file.
%
%
%   Execute
%   SUPPORTED_ROBOTS to see a list of all supported robots.
%
%   The robot data structure consists of the following fields:
%
%	See also SUPPORTED_ROBOTS.
%
%   Author: Arturo Gil. Universidad Miguel Hern�ndez de Elche. email:
%   arturo.gil@umh.es date:   02/01/2012
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
%function robot = load_robot(manufacturer, version)

function robot = load_robot(varargin)

global configuration %robot

%process variable number of arguments
%if the function is called with no arguments then a dialog
%box is used to retrieve the absolute path of the parameters.m file.
if nargin==0
   [FileName,PathName,FilterIndex] = uigetfile({'*.m','Parameter-files (*.m)'},'Pick a robot parameters.m file', 'MultiSelect', 'off');
   full_name=[PathName '/' FileName];
   cd(PathName);
elseif nargin==1 % the relative path is expressed in a single argument%load base libpath
    cd(configuration.libpath);
    % go to the directory where the robot is stored
    temp_path = [ configuration.libpath '/robots/' varargin{1}];
    cd(temp_path);

    %make a command to evaluate the parameter.m file at this directory
    full_name = [temp_path '/parameters.m'];
    
elseif nargin==2 % the relative path is expressed in the 'manufacturer', 'version' style
    manufacturer = varargin{1};
    version = varargin{2};
    %load base libpath
    cd(configuration.libpath);
    % go to the directory where the robot is stored
    temp_path = [ configuration.libpath '/robots/' manufacturer '/' version];
    cd(temp_path);

    %make a command to evaluate the parameter.m file at this directory
    full_name = [temp_path '/parameters.m'];
end

%initialize robot data structure
%robot=[];



%Eval parameters file for the selected robot
%eval(command);
%check whether the parameters.m file exists at the required location
if exist(full_name,'file')==2
    robot=eval('parameters');
    
    %now draw robot
    drawrobot3d(robot, zeros(1,robot.DOF));
    %default view
    axis(robot.axis);
else %if there's no parameters.m file
    fprintf('ERROR: No valid parameters.m file found. Please check the following directory')
    full_name
end





