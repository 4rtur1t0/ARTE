%%%%%%%%%%%%%%%%%%
%   LINK 0: BASE
%%%%%%%%%%%%%%%%%%
filename='link0.stl'; %base
cyl_radius=0.075;
cyl_height = 0.290;
precision = 10; % increase to obtain a more accurate drawing

%create a unit height cylinder with 100 points. Radius 75
[X,Y,Z] = cylinder([cyl_radius], precision);
%Multiply Z by height
Z=Z*cyl_height;

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
cyl_height = 0.150;

%create a unit height cylinder with 100 points. Radius 75
[X,Y,Z] = cylinder([cyl_radius], precision);
% draw Z correspondingly
Z(1,:)=Z(1,:) -cyl_height/2;
Z(2,:)=Z(2,:).*(cyl_height/2);

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');

%now create a disk, bottom
radius = linspace(0,cyl_radius,precision); % For ten rings
theta = (pi/180)*[0:15:360]; % For eight angles
[R,T] = meshgrid(radius,theta); % Make radius/theta grid
X = R.*cos(T); % Convert grid to cartesian coordintes
Y = R.*sin(T);

%append this solid to already created file, bottom disk
surf2stl(filename, X, Y, (-cyl_height/2).*ones(size(X,1), size(X,2)), 'ascii', 'a+');
%top disk
surf2stl(filename, X, Y, (cyl_height/2).*ones(size(X,1), size(X,2)), 'ascii', 'a+');


%%%%%%%%%%%%%%%%%%
%   LINK 2
%%%%%%%%%%%%%%%%%%
filename='link2.stl'; 
%cylinder 1
cyl_radius=0.03;
cyl_height = 0.1;
[X,Y,Z] = cylinder([cyl_radius], precision);
% draw Z correspondingly
Z(1,:)=Z(1,:) -cyl_height/2;
Z(2,:)=Z(2,:).*(cyl_height/2);

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');

cyl_radius=0.05;
cyl_height = 0.450;
%draw arm, swap X and Z
[Z,Y,X] = cylinder([cyl_radius cyl_radius*0.8], precision);
% draw Z correspondingly
X(1,:)=X(1,:) -cyl_height;
X(2,:)=X(2,:).*0;

%surf(X, Y, Z)

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'a+');



%%%%%%%%%%%%%%%%%%
%   LINK 3
%%%%%%%%%%%%%%%%%%
filename='link3.stl'; 
%cylinder 1
% cyl_radius=0.03;
% cyl_height = 0.1;
% [X,Y,Z] = cylinder([cyl_radius], precision);
% % draw Z correspondingly
% Z(1,:)=Z(1,:) -cyl_height/2;
% Z(2,:)=Z(2,:).*(cyl_height/2);
% 
% %Save in stl format, create new file
% surf2stl(filename, X, Y, Z, 'ascii', 'w');

cyl_radius=0.04;
cyl_height = 0.50;
%draw arm, swap X and Z
[X, Y, Z] = cylinder([cyl_radius cyl_radius*0.8], precision);
% draw Z correspondingly

Z(2,:)=Z(2,:).*(cyl_height);

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'a+');


%%%%%%%%%%%%%%%%%%
%   LINK 4
%%%%%%%%%%%%%%%%%%
filename='link4.stl'; 

cyl_radius=0.03;
cyl_height = 0.07;
%draw arm, swap X and Z
[X, Y, Z] = cylinder([cyl_radius], precision);
% draw Z correspondingly
 Z(1,:)=Z(1,:) -cyl_height/2;
 Z(2,:)=Z(2,:).*(cyl_height/2);

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');



%%%%%%%%%%%%%%%%%%
%   LINK 5
%%%%%%%%%%%%%%%%%%
filename='link5.stl'; 

cyl_radius=0.03;
cyl_height = 0.07;
%draw arm, swap X and Z
[X, Y, Z] = cylinder([cyl_radius], precision);
% draw Z correspondingly
 Z(2,:)=Z(2,:).*(cyl_height);

%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');



%%%%%%%%%%%%%%%%%%
%   LINK 6
%%%%%%%%%%%%%%%%%%
filename='link6.stl'; 

cyl_radius=0.05;
cyl_height = 0.01;
%draw arm, swap X and Z
[X, Y, Z] = cylinder([cyl_radius], precision);
% draw Z correspondingly
 Z(1,:)=Z(1,:) -cyl_height;
 Z(2,:)=Z(2,:).*0;
%Save in stl format, create new file
surf2stl(filename, X, Y, Z, 'ascii', 'w');

%now create a disk, bottom
radius = linspace(0,cyl_radius,precision); 
theta = (pi/180)*[0:15:360]; 
[R,T] = meshgrid(radius,theta); % Make radius/theta grid
X = R.*cos(T); 
Y = R.*sin(T);

%append this solid to already created file, bottom disk
surf2stl(filename, X, Y, (-cyl_height).*ones(size(X,1), size(X,2)), 'ascii', 'a+');
%append this solid to already created file, bottom disk
surf2stl(filename, X, Y, 0.*ones(size(X,1), size(X,2)), 'ascii', 'a+');
