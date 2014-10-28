%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT=READ_GRAPHICS(ROBOT)	
% Reads the 3D graphics files associated to the robot.
% Each robot has N+1 graphics files stored at its directory: robot/abb/IRB140
% link0.stl, link1.stl, ..., link6.stl, where link0.stl defines the robot
% base, link1.stl, the first link connecting the base and link 2 etc.
% Each graphics file is defined in STL (Standard Tessellation Language)
% format, supported by many 3D software packages:
%   http://en.wikipedia.org/wiki/STL_%28file_format%29
%
% Prior to the call of READ_GRAPHICS, the variable
% robot.graphical.has_graphics has to be set to one.
%
% ROBOT=READ_GRAPHICS(ROBOT), stores the current robot Faces and vertices
% of each file in the variables:
%   
%  robot.graphical.link{1}.f --> faces robot base, 
%  robot.graphical.link{1}.v --> vertices robot base,
%  robot.graphical.link{2}.f --> faces link 1, 
%  robot.graphical.link{2}.v --> vertices link 1...
%
%  the vector f defines the position of a set of points that represent each
%  robot link in its current DH reference system. The units of f are
%  meters.
%
%  READ_GRAPHICS calls the function STL_READ, in charge of reading each of
%  the files associated to the robot.
%
% See also DRAW_LINK, DRAW_PATCH, STL_READ.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   05/02/2012
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
function robot=read_graphics(robot)

%degrees of freedom
n=robot.DOF;

if robot.graphical.has_graphics    
    
    full_name = [robot.path '/link0.stl'];
    
    %check whether there exist graphics files for the robot
    if exist(full_name, 'file')~=2
        disp('NO GRAPHICS FILE FOUND, not reading graphics');
        disp('Please, remember to place link0.stl, link1.stl at each robot directory');
        disp('Check the following directory: ')
        robot.path
        robot.graphical.has_graphics=0
        return;
    end
    pwd
    for i=0:n,
        link_name=sprintf('/link%d.stl', i);
        fprintf('\nReading link %d\n %s\n', i, [robot.path link_name]);
        %load link 0, base
        [fout, vout, cout] = stl_read([robot.path link_name]);
        robot.graphical.link{i+1}.f = fout;
        robot.graphical.link{i+1}.v = vout;
        %robot.graphical.link{i+1}.c = cout+robot.graphical.color;
    end
    %load a default color if not specified
    if robot.graphical.color(1) > 1.0
        robot.graphical.color = [204 51 0]./255;
        disp('NO VALID COLOR FOUND, PLEASE SPECIFY robot.graphical.color as [0.5 0.6 0.7]');
    end
end