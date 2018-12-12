function [nota1, nota2]=check_my_kinematics(robot)


   %q=zeros(1, robot.DOF) + 0.05*rand(1, robot.DOF);
   q=[0.1 0.1 0.1 0.1 0.1 0.1] + 0.05*rand(1, robot.DOF);
   
   %Now compute direct kinematics for this position q
   T = directkinematic(robot, q);

   %Set to zero if you want to see the robot transparent
   robot.graphical.draw_transparent=1;
   robot.graphical.draw_axes=1;

   qinv = inversekinematic(robot, T);


resultado_q = [];
resultado_T = [];

for i=1:size(qinv,2),
    Ti = directkinematic(robot, qinv(:,i)); %Ti is constant for the different solutions
    
    resultado_q= [resultado_q sum(qinv(:,i)-q')];
    resultado_T= [resultado_T sum(sum(T-Ti))];
            
    %now draw the robot to see the solution
    drawrobot3d(robot, qinv(:,i))
    
    pause(1)
end


indice1 = find(abs(resultado_q) < 0.05);
indice2 = sum(abs(resultado_T));


if size(indice1,2)==0
   nota1=0;
else 
   nota1=10;
end

if indice2>0.01
    nota2=0;
else
    nota2=10;    
end








