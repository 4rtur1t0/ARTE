function [p1,p2]=calculo_puntos2(robot)%p1,p2

%PUNTO INICIAL 
%q1=[-0.138 -0.984 -1.274 0.105 -0.021 2.108]; %posicion incial del robot = posicion inical de la caja
%q1=[-0.132 -0.967 -1.289 0.279 -0.021 0];
q1=[-0.2827 -0.1396 -1.85 -1.1345 0.3037 0];
T1=directkinematic(robot,q1);

%T2=T1*[eye(3),[0,0,-0.2]';0,0,0,1]*[[1,0,0;0,cos(pi),-sin(pi);0,sin(pi),cos(pi)],[0,0,0]';0,0,0,1];
betha1=asin(T1(1,3));
gamma1=asin(-T1(1,2)/cos(betha1));
alpha1=acos(T1(3,3)/cos(betha1));
p1=[T1(1,4) T1(2,4) T1(3,4) alpha1 betha1 gamma1]; %posicion inicial (de euler)

%PUNTO FINAL 1
q2=[-0.3142 -0.2443 -2.0769 -1.314 0 0];
%creo q el bueno q2=[-0.1396 -0.8029 -1.2217 -0.2967 -0.0209 0];
%q2=[-0.1257 -1.4520 -0.3790 0 -0.2010 0];
T2=directkinematic(robot,q2);
betha2=asin(T2(1,3));
gamma2=asin(-T2(1,2)/cos(betha2));
alpha2=acos(T2(3,3)/cos(betha2));
p2=[T2(1,4) T2(2,4) T2(3,4) alpha2 betha2 gamma2];


%PUNTO FINAL 2

%PUNTO FINAL 3

end