%%%%%%%%%%%%%%%%%%
%   LINK 0: BASE
%%%%%%%%%%%%%%%%%%
%add two basic shapes to this file
filename='link0_test.stl'; %base
cyl_radius=0.1;
cyl_height = 0.05;
precision = 20; % increase to obtain a more accurate drawing

%create a unit height cylinder with 100 points. Radius 75
[X,Y,Z] = cylinder([cyl_radius], precision);
%Multiply Z by height
Z=Z*cyl_height;


%translation of the points
X=X+0.4;
Y=Y+0.2;
Z=Z+0.2


%Save in stl format, append to file
surf2stl(filename, X, Y, Z, 'ascii', 'a+');


%make a box using the cylinder function
cyl_radius=0.15;
cyl_height = 0.1;
precision = 4; % just 4 makes a box, 3 a triangle
%create the box using the cylinder function, easy
[X,Y,Z] = cylinder([cyl_radius], precision);
%Multiply Z by height
Z=Z*cyl_height;

%translation of the points
X=X+1.0;
Y=Y+0.2;
Z=Z+0.2


%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'a+');
