%%%%% Representacion en esfera (radio variable, posicion (0,0,0),escribe por el "polo sur"):

function [xyz, lin]=en_esfera(P2)
    global radio;
    global precision;
    
    %precision=0.001;
    P=[P2(1) -P2(2) -radio];
    V=[0,0,radio]-P;
    for i=0:1:10000;
        recta(i+1,:)=P+V.*(i*precision/10);
        if abs((recta(i+1,1))^2+recta(i+1,2)^2+(recta(i+1,3))^2-radio^2) < precision
            xyz=recta(i+1,:);
            %display('exito')
            lin=recta;
            break
        end
    end
%     figure
%     hold on
%     [x,y,z] = sphere;
%     surf(x,y,z)
%    plot3(recta(:,1),recta(:,2),recta(:,3),'*')
    %hold on
    %plot3(xyz(1),xyz(2),xyz(3),'*')
end
