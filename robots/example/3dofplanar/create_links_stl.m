%%%%%%%%%%%%%%%%%%
%   LINK 0: BASE
%%%%%%%%%%%%%%%%%%
filename='link0.stl'; %base
cyl_radius=0.05;
cyl_height = 0.1;
precision = 20; % increase to obtain a more accurate drawing

%create a unit height cylinder with 100 points. Radius 75
[X,Y,Z] = cylinder([cyl_radius], precision);

%Multiply Z by height
Z=Z*2*cyl_height-cyl_height;

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');

%now create a disk, bottom
radius = linspace(0,cyl_radius,precision); % For ten rings
theta = (pi/180)*[0:15:360]; % For eight angles
[R,T] = meshgrid(radius,theta); % Make radius/theta grid
X = R.*cos(T); % Convert grid to cartesian coordintes
Y = R.*sin(T);

%append this solid to already created file, bottom disk
surf2stl(filename, X, Y, 0.*X, 'ascii', 'a+');
%top disk
surf2stl(filename, X, Y, cyl_height.*ones(size(X,1), size(X,2)), 'ascii', 'a+');



%%%%%%%%%%%%%%%%%%
%   LINK 1
%%%%%%%%%%%%%%%%%%
filename='link1.stl'; 
cyl_radius=0.05;
cyl_height = 1;
precision = 20; % increase to obtain a more accurate drawing

%create a unit height cylinder with 100 points. Radius 75
[Z,Y,X] = cylinder([cyl_radius], precision);
%Multiply Z by height
X=-X*cyl_height;

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');


%%%%%%%%%%%%%%%%%%
%   LINK 2
%%%%%%%%%%%%%%%%%%
filename='link2.stl';
cyl_radius=0.05;
cyl_height = 1;
precision = 20; % increase to obtain a more accurate drawing

%create a unit height cylinder with 100 points. Radius 75
[Z,Y,X] = cylinder([cyl_radius], precision);
%Multiply Z by height
X=-X*cyl_height;

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');


%%%%%%%%%%%%%%%%%%
%   LINK 3
%%%%%%%%%%%%%%%%%%
filename='link3.stl';
cyl_radius=0.05;
cyl_height = 1;
precision = 20; % increase to obtain a more accurate drawing

%create a unit height cylinder with 100 points. Radius 75
[Z,Y,X] = cylinder([cyl_radius], precision);
%Multiply Z by height
X=-X*cyl_height;

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');
