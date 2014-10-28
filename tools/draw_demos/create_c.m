%sample file to create a solid cylinder with two disks on both sides
cyl_radius=0.75;
cyl_height = 0.290;

%create a unit height cylinder with 100 points. Radius 75
[X,Y,Z] = cylinder([cyl_radius], 100);
%Multiply Z by height
Z=Z*cyl_height;

%Save in stl format, create new file
surf2stl('link0.stl', X, Y, Z, 'ascii', 'w');

%now create a disk, bottom
radius = linspace(0,cyl_radius,10); % For ten rings
theta = (pi/180)*[0:10:360]; % For eight angles
[R,T] = meshgrid(radius,theta); % Make radius/theta grid
X = R.*cos(T); % Convert grid to cartesian coordintes
Y = R.*sin(T);
%Z = 0.*X; %(corresponding values of Z in terms of X and Y)
%surf(X,Y,Z) 
%append this solid to already created file, bottom disk
surf2stl('link0.stl', X, Y, 0.*X, 'ascii', 'a+');
%top disk
surf2stl('link0.stl', X, Y, cyl_height.*ones(size(X,1), size(X,2)), 'ascii', 'a+');
%top disk
%Z = cyl_height.*X; %(corresponding values of Z in terms of X and Y)
%append
