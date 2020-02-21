function [coord,R]=calcgrip(Tr,Tc)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%EN FUNCION DE DONDE ESTE LA CAJA Y DONDE ESTE EL ROBOT,
%CALCULA QUE LADO DE LA CAJA ESTA A MENOS DISTANCIA DE LA 
%POSICION ACTUAL, Y DEVUELVE POR DONDE TIENE QUE COGER LA CAJA
%Y CON QUE ORIENTACION

    Tc(3,4)=Tc(3,4)+0.05;
    Tc1=Tc(1:3,4);
    Tc2=Tc(1:3,4);
    Tc3=Tc(1:3,4);
    Tc4=Tc(1:3,4);
    Tc1(1,4)=Tc(1,4)+0.14;
    dist(1,1)=norm(Tc1-Tr(1:3,4));
    Tc2(1,4)=Tc(1,4)-0.14;
    dist(2,1)=norm(Tc2-Tr(1:3,4));
    Tc3(1,4)=Tc(2,4)+0.10;
    dist(3,1)=norm(Tc3-Tr(1:3,4));
    Tc4(1,4)=Tc(2,4)-0.10;
    dist(4,1)=norm(Tc4-Tr(1:3,4));
    [M,I]=min(dist);
    if I==1
        coord=[0.14;0;0.05];
        R=[0 0 1 ;0 1 0;1 0 0];
    end
    if I==2
        coord=[-0.14;0;0.05];
        
        R=[0 0 -1 ;0 1 0;-1 0 0];
    end
    if I==3
        coord=[0;0.10;0.05];
        R=[0 -1 0 ;0 0 1; -1 0 0];
        
    end
    if I==4
        coord=[0;-0.10;0.05];
        R=[0 1 0 ;0 0 -1; 1 0 0];
    end
    
    
end
    
    



