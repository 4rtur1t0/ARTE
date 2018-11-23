%   GENERATE_GAUSSIAN(mu, sigma) 
%   Using the BOX-MULLER transformation to generate pseudo-gaussian numbers
%   
%   https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   05/01/2012

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
function delta=generate_gaussian(mu, sigma)
    %min double number in Matlabe
	epsilon = realmin; 
	two_pi = 2.0*pi;
    
    %using two global variables
    global z1 
    global generate
    generate = ~generate;
    
	if generate
        delta=z1 * sigma + mu;
    end

    while 1        
       u1 = rand();
	   u2 = rand();
       if ( u1 > epsilon )
            break
       end
    end  

	
	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	delta= z0 * sigma + mu;
