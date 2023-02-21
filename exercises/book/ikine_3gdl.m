% DIRECT JACOBIAN DEMO
function cinematica_inversa_3gdl
% definimos la posición y orientación del extremo en 2D
phi = pi/8
px = 2
py = 2
% definimos una matriz T
T = [cos(phi) -sin(phi) 0 px;
     sin(phi) cos(phi)  0 py;
     0 0 1 0;
     0 0 0 1];
 
 q = ikine3DOF(T)
 
 
 function q = ikine3DOF(T)
 L1 = 1;
 L2 = 1;
 L3 = 1; % 1m, parámetro del robot

 % hallamos pm
 p = T(1:3, 4);

 
%find angle Phi
x3 = T(1:3,1);

cphi = x3'*[1 0 0]';
sphi = x3'*[0 1 0]';
phi = atan2(sphi, cphi);


% pm
pm = p - L3*x3
 

%Distance of the point to the origin. 
R= sqrt(pm(1)^2+pm(2)^2);

if R > (L1+L2)
   disp('\ninversekinematic_3dofplanar: unfeasible solution. The point cannot be reached'); 
end

%compute geometric solution
beta = atan2(pm(2),pm(1)); 
gamma = real(acos((L1^2+R^2-L2^2)/(2*R*L1)));
delta = real(acos((L1^2+L2^2-R^2)/(2*L1*L2)));

%arrange possible combinations for q(1) and q(2) 
%elbow down     elbow up solutions
q =[beta+gamma beta-gamma;
    delta-pi   pi-delta];

%in this case, phi = q(1) + q(2) + q(3) and
%q(3) can be computed as q(3) = phi - q(1) - q(2) 
%corresponding to each of the previous solutions, a unique q(3) can be
%computed for each case
for i=1:2 %iterate through columns 
    q(3,i) = phi - q(1,i) - q(2,i); 
end
