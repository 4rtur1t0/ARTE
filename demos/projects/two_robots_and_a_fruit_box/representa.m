function representa(robot,qt1,qt2,num,coord,num2)
%REPRESENTA LOS DOS ROBOTS Y LA CAJA SIGUIENDO LA TRAYECTORIA

for i=1:size(qt1,2)
    
    if num==2
        
        T= directkinematic(robot.robot1, qt1(:,i));
        Pc = T(1:3,4);
        tangente=atan(T(2,3)/T(1,3));
        robot.robot1.equipment{1}.T0(1:3,4) = Pc;
        robot.robot1.equipment{1}.T0(3,4)=robot.robot1.equipment{1}.T0(3,4)-coord(3,1);
        if num2==1
       
         robot.robot1.equipment{1}.T0(2,4)=robot.robot1.equipment{1}.T0(2,4)+norm(coord(1:2,1))*sin(tangente);
         robot.robot1.equipment{1}.T0(1,4)=robot.robot1.equipment{1}.T0(1,4)+norm(coord(1:2,1))*cos(tangente);
         
        robot.robot1.equipment{1}.T0(1:3,1:3)=T(1:3,1:3)*[0 -1  0;
                                                          1  0  0;
                                                          0  0  1];
        else
           robot.robot1.equipment{1}.T0(2,4)=robot.robot1.equipment{1}.T0(2,4)-norm(coord(1:2,1))*sin(tangente);
         robot.robot1.equipment{1}.T0(1,4)=robot.robot1.equipment{1}.T0(1,4)+norm(coord(1:2,1))*cos(tangente);
        end
        
        
    end
      draw2robots(robot.robot1,qt1(:,i),robot.robot2,qt2(:,i))
      
      
end



end