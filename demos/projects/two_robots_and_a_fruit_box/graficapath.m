function graficapath(qt1,qt2,robot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LE PASAS TODAS LAS POSICIONES ARTICULARES QUE TIENE EL ROBOT EN UNA 
%TRAYECTORIA Y HACE UNA GRAFICA EN 3D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:2
    for j=1:size(qt1,2)
        T=directkinematic(robot.robot1,qt1(:,j));
       p1(j,:)=T(1:3,4)';
        
    end
    for j=1:size(qt2,2)
        
        T=directkinematic(robot.robot2,qt2(:,j));
       p2(j,:)=T(1:3,4)';
        
    end
end

 figure, title('Trayectorias'), hold on
 plot3(p1(:,1),p1(:,2),p1(:,3),'x');
 plot3(p2(:,1),p2(:,2),p2(:,3),'o');



end