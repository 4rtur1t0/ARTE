%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [F, V, C] = STL_READ(filename)
% Read all the vertex contained in the FILENAME STL .
% (Standard Tessellation Language) file 
% Refer to http://en.wikipedia.org/wiki/STL_%28file_format%29 for more
% details.
% 
% [F, V, C] = STL_READ(filename) returns:
%   F defines the faces in numbers.
%   V defines the cartesian coordinates of each of the faces. These coordinates are
%       expressed in meters in the links DH reference frame.
%   C is not currently used.
%
%  See also STLWRITE, SURF2STL 
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
function [F, V, C] = stl_read(filename)
V=[];
F=[];

file = fopen(filename, 'r');

%first, get all cartesian points
[V, point_numbers]=get_cartesian(file,  'vertex');

fclose(file);

n=size(V,1);

F = [(1:3:n)' (2:3:n)' (3:3:n)'];
C=zeros(n,1);


function [data, point_numbers] =get_cartesian(file, tag)

%tag length
n=length(tag);
data=[];
point_numbers = [];
%get every cartesian point
while 1    
    tline = fgets(file);
    
    if tline == -1
        fprintf('EndOfFile found... ');
        error = 1;        
        break;
    end

    %#num
    [temp, remain] = strtok(tline, '  '); 
    
    if strncmp(temp, tag, length(tag))
        %Read data 
        [a] = sscanf(remain,'%f');  
        
        data = [data; a'];      
    end       
end
