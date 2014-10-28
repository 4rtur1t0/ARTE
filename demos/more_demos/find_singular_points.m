% LOOK FOR SINGULAR POINTS IN JOINT SPACE

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

function singular_points = find_singular_points(param)

singular_points=[];

q=[0 0 0 0];

teta = eval(param.DH.theta);
d = eval(param.DH.d);
a = eval(param.DH.a);
alfa = eval(param.DH.alpha);

%J = eval(param.J)


q1=-pi:0.05:pi;
q2=-pi:0.05:pi;

for i=1:length(q1),
    fprintf('\nIteration %d of %d', i, length(q1))
   for j=1:length(q2),
        q=[q1(i) q2(j) 0];
        J = eval(param.J);
       
        val = det(J);
        if abs(val) < 0.0001
            singular_points = [singular_points q'];
        end
   end
end