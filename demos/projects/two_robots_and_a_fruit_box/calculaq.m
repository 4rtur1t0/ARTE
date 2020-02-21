function [qt1,qt2]=calculaq(qi1,qf1,qi2,qf2,delta_t,orden,t,robot)
if orden==3   
for i=1:6
    [qt1(i,:)]=interpola3(delta_t,qi1(i,1),qf1(i,1),t);
end

for i=1:6
    [qt2(i,:)]=interpola3(delta_t,qi2(i,1),qf2(i,1),t);
end

end

if orden==2
    for i=1:6
    [qt1(i,:)]=interpola2(delta_t,qi1(i,1),qf1(i,1),t);
    end

for i=1:6
   [ qt2(i,:)]=interpola2(delta_t,qi2(i,1),qf2(i,1),t);
end
    
    
end

if orden==1
    
    a=2;
    %robot1%
    Ti1=directkinematic(robot.robot1,qi1);
    Tf1=directkinematic(robot.robot1,qf1);
    vec=(Tf1(1:3,4)-Ti1(1:3,4))';
    dist=norm(vec);
    V=[vec(1,1);vec(1,2);vec(1,3);0;0;0];
    V=V/t;
    qt1(:,1)=qi1;
    n=t/delta_t;
    delta_dist=dist/n;
    
    for i=0:delta_dist:dist
     
     Jn=compute_jacobian(robot.robot1,qt1(:,a-1));
  
     qt1(:,a)=qt1(:,a-1)+delta_t*inv(Jn)*V;
     
    a=a+1;
    end
    
    %robot2%
    
    a=2;
  
    Ti2=directkinematic(robot.robot2,qi2);
    Tf2=directkinematic(robot.robot2,qf2);
    vec=(Tf2(1:3,4)-Ti2(1:3,4))';
    dist=norm(vec);
    V=[vec(1,1);vec(1,2);vec(1,3);0;0;0];
    V=V/t;
    qt2(:,1)=qi2;
    n=t/delta_t;
    delta_dist=dist/n;
    for i=0:delta_dist:dist
    Jn=compute_jacobian(robot.robot2,qt2(:,a-1));
 
    qt2(:,a)=qt2(:,a-1)+delta_t*inv(Jn)*V;
    a=a+1;
    end
    
    
end


end