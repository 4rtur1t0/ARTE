function [p1,p1_2,p2,p2_2]=calculo_puntos(robot)%p1,p2

%PUNTO INICIAL 
%q1=[-0.138 -0.984 -1.274 0.105 -0.021 2.108]; %posicion incial del robot = posicion inical de la caja
%%% q1=[-0.132 -0.967 -1.289 0.279 -0.021 0];
q1=[-0.2827 -0.1396 -1.85 -1.1345 0.3037 0];
%q1_robot2=[0.1440    0.9641    1.3023   -0.0121    0.0544   -1.0145];
q1_robot2=[0.2829    0.1391    1.8535    1.1314   -0.3008   -0.0051];
T1=directkinematic(robot,q1);
T1_2=directkinematic(robot,q1_robot2);
%T2=T1*[eye(3),[0,0,-0.2]';0,0,0,1]*[[1,0,0;0,cos(pi),-sin(pi);0,sin(pi),cos(pi)],[0,0,0]';0,0,0,1];
betha1=asin(T1(1,3));
gamma1=asin(-T1(1,2)/cos(betha1));
alpha1=acos(T1(3,3)/cos(betha1));
p1=[T1(1,4) T1(2,4) T1(3,4) alpha1 betha1 gamma1]; %posicion inicial (de euler)
betha1_2=asin(T1_2(1,3));
gamma1_2=asin(-T1_2(1,2)/cos(betha1_2));
alpha1_2=acos(T1_2(3,3)/cos(betha1_2));
p1_2=[T1_2(1,4) T1_2(2,4) T1_2(3,4) alpha1_2 betha1_2 gamma1_2];
%PUNTO FINAL 1
%q2=[-0.119 0.405 -1.626 -0.74 0 2.011];
%buueno creo q2=[-0.1396 -0.8029 -1.2217 -0.2967 -0.0209 0];
q2=[-0.314 -0.2443 -2.0769 -1.314 0 0];
T2=directkinematic(robot,q2);
betha2=asin(T2(1,3));
gamma2=asin(-T2(1,2)/cos(betha2));
alpha2=acos(T2(3,3)/cos(betha2));
p2=[T2(1,4) T2(2,4) T2(3,4) alpha2 betha2 gamma2];
%q2_2=[0.1498   -0.4106    1.6997    0.8706    0.0273   -2.0933];
%bueno q2_2=[0.1517    0.8126    1.3026    0.1152    0.0528   -1.3151];
q2_2=[0.3132    0.1414    2.1679    0.7992   -0.0059   -0.4911];
T2_2=directkinematic(robot,q2_2);
betha2_2=asin(T2_2(1,3));
gamma2_2=asin(-T2_2(1,2)/cos(betha2_2));
alpha2_2=acos(T2_2(3,3)/cos(betha2_2));
p2_2=[T2_2(1,4) T2_2(2,4) T2_2(3,4) alpha2_2 betha2_2 gamma2_2];
end