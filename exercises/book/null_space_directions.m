% Copyright (C) 2016, by Arturo Gil Aparicio
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
function null_space_directions
close all
th = pi/4;
lambda1=1;
lambda2=2;

A=[ lambda1*cos(th)  -lambda2*sin(th) ; 
    lambda1*sin(th)   lambda2*cos(th)];

% A=[-6 3; 
%     4 5];


% set of vectors
xs = [];
ys = [ ];
for phi=0:0.1:2*pi
    x = [cos(phi) sin(phi)]';
    y = A*x;
    xs = [xs x];
    ys = [ys y];
end
figure, plot(xs(1,:), xs(2,:), 'b.'), hold, axis equal
plot(ys(1,:), ys(2,:), 'r.')

[V, D] = eig(A);

% svd of A, such that A = U*S*V'
[U,S,V] = svd(A)
% basic properties
%As = U*S*V'
%iA = V*[1/S(1,1) 0; 0 1/S(2,2)]*U'

% plot eigenvectors
%plot(