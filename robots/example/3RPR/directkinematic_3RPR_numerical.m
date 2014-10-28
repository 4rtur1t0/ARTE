% Mecanismo 3RPR
% 4º Grado Ingeniería Electrónica y Automática Industrial
% Asignatura: Robótica
% 2013/2014

% Merlos Ortega, Juan Antonio
% Pérez Sotobal, Enrique

function T=directkinematic_3RPR_numerical(robot, theta, threshold, max_iter)

global xQ yQ xR yR h a
L=2.5; 

%posición de los puntos de coordenadas de Q y R
xQ=L; yQ=0; xR=L/2; yR=L;

h=0.5; 
a=1;

%evaluar parámetros de convergencia
if ~exist('threshold','var')
    threshold=0.001;
end

if ~exist('max_iter','var')
    max_iter=50;
end


%parámetros de entrada para la cinemática directa
th1=theta(1); 
th2=theta(2); 
th3=theta(3); 

theta = [th1 th2 th3];

%Valores iniciales de beta (beta = [xA yA Phi b1 b2 b3])
beta=[1 0.5774 0 0.155 0.155 0.364]';

y=[0 0 0 0 0 0]';

fs=[];

for i=1:max_iter,    
    f=compute_gamma_values(beta,theta);
    J=compute_jacobian_beta(beta,theta);
    
    delta=inv(J'*J)*J'*(y-f);
    
    %actualizar beta
    beta = beta + delta;   
    
    sumofsqes=(y-f)'*(y-f);
    fs=[fs sumofsqes];
    
    if sumofsqes < threshold  
        fprintf('\ndirectkinematic_3RPR::Solucion encontrada en %d iteraciones',i);
        fprintf('\nA tener en cuenta que solo se devuelve una de las posibles soluciones');
        break;
    end
end

if i==max_iter
  fprintf('ERROR:: directkinematic_3RPR::No puede converger en %d iteraciones',i);
end

%Construir solución desde los parámetros beta
T=eye(4);
T(1,4)=beta(1);
T(2,4)=beta(2);
T(1,1)=cos(beta(3));
T(2,1)=sin(beta(3));
T(1,2)=-sin(beta(3));
T(2,2)=cos(beta(3));


%dibujar la solución dada
q=[th1 beta(4) th2 beta(5) th3 beta(6)];
drawrobot3d(robot, q), pause(2);

figure, plot(fs), title('Sum of squares vs. iteration step')



function f=compute_gamma_values(beta, theta)

global xQ yQ xR yR a h


xA=beta(1);
yA=beta(2);
Phi=beta(3);
b1=beta(4);
b2=beta(5);
b3=beta(6);

th1=theta(1);
th2=theta(2);
th3=theta(3);

%Ecuaciones de cierre para el mecanismo 3RPR 
f1=eval('xA-(a+b1)*cos(th1)');
f2=eval('yA-(a+b1)*sin(th1)');
f3=eval('xA-xQ-(a+b2)*cos(th2)+ h*cos(Phi)');
f4=eval('yA-yQ-(a+b2)*sin(th2)+ h*sin(Phi)');
f5=eval('xA-xR-(a+b3)*cos(th3)+ h*cos(Phi+pi/3)');
f6=eval('yA-yR-(a+b3)*sin(th3)+ h*sin(Phi+pi/3)');


f=[f1 f2 f3 f4 f5 f6]';


%Calcular la Jacobiana respecto a los valores xA yA Phi, b1, b2, b3
function J=compute_jacobian_beta(beta, theta)

global h

%xA=beta(1);
%yA=beta(2);
Phi=beta(3);
%L1=beta(4);
%L2=beta(5);
%L3=beta(6);


th1=theta(1);
th2=theta(2);
th3=theta(3);

%Calculo de la Jacobiana
%Gamma1 
J11=1;
J12=0;
J13=0;
%J14=L1*sin(th1);
J14=-cos(th1);
J15=0;
J16=0;

%Gamma2 
J21=0;
J22=1;
J23=0;
%J24=-L1*cos(th1);
J24=-sin(th1);
J25=0;
J26=0;
  
%Gamma3
J31=1;
J32=0;
J33=-h*sin(Phi);
J34=0;
%J35=L2*sin(th2);
J35=-cos(th2);
J36=0;

%Gamma4
J41=0;
J42=1;
J43=h*cos(Phi);
J44=0;
%J45=-L2*cos(th2);
J45=-sin(th2);
J46=0;

%Gamma5
J51=1;
J52=0;
J53=-h*sin(Phi+pi/3);
J54=0;
J55=0;
%J56=L3*sin(th3);
J56=-cos(th3);

%Gamma6 
J61=0;
J62=1;
J63=h*cos(Phi+pi/3);
J64=0;
J65=0;
%J66=-L3*cos(th3);
J66=-sin(th3);

J=[J11 J12 J13 J14 J15 J16;
   J21 J22 J23 J24 J25 J26;
   J31 J32 J33 J34 J35 J36;
   J41 J42 J43 J44 J45 J46;
   J51 J52 J53 J54 J55 J56;
   J61 J62 J63 J64 J65 J66];
