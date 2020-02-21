function qt=interpoladorcirc(robot,qi,grados,grip,num)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%USA LA JACOBIANA PARA HACER UN GIRO
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ti=directkinematic(robot,qi);
Pi=Ti(1:2,4)';
coord(1,:)=Pi;
delta_t=0.1;
a=2;
w=(grados*pi/3)/(pi/2);
w=[0 0 w];
qt(:,1)=qi;

switch grip%EN FUNCION DE DONDE ESTE COGIENDO LA CAJA CAMBIA
    %YA QUE AL ESTAR RESPECTO EL SISTEMA GLOBAL, GIRAR 90º NO SIEMPRE ES
    %GIRAR DE 0 A 90º
    
   case 0.1
       grado=0;
        
   case -0.1
       grado=0;
   case 0.14
       grado=pi/2;
   otherwise 
       grado=-pi/2;
end


for i=grado:delta_t:grado+grados-0.1
    
    if num==1%ROBOT1
    coord(a,1)=grip*cos(i)+coord(a-1,1);%;+grip-0.4
    coord(a,2)=grip*sin(i)+coord(a-1,2);%;+grip+0.1
    else%ROBOT2
    coord(a,1)=grip*cos(i)+coord(a-1,1);
    coord(a,2)=grip*sin(i)+coord(a-1,2);
    end
    vl=coord(a-1,:)-coord(a,:);
    V=[vl(1,1);vl(1,2);0;w'];
    Jn=compute_jacobian(robot,qt(:,a-1)); 
   
    qt(:,a)=qt(:,a-1)+delta_t*inv(Jn)*V;
   
    a=a+1;
end


end
