function Tin=inicial(yy,T,robot)
tam=length(T);
T1=yy*T{1};
global radio;
global intervalos;
global Tinicial;
%Tinicial=directkinematic(robot,[0 0 0 0 0 0]);
 x=Tinicial(1:3,1);
 y=Tinicial(1:3,2);
 z=Tinicial(1:3,3);
 Tin{1}=Tinicial;
 pmedio=[T1(1,4),T1(2,4),(T1(3,4)- radio)/2];
 
 for j=2:1:(intervalos*2);
    if j<=intervalos
        inicial(1,j)=(j-1)*(pmedio(1)-Tinicial(1,4))/intervalos + Tinicial(1,4);
        inicial(2,j)=(j-1)*(pmedio(2)-Tinicial(2,4))/intervalos + Tinicial(2,4);
        inicial(3,j)=(j-1)*(pmedio(3)-Tinicial(3,4))/intervalos + Tinicial(3,4);

       %Ejes tramo inicial
       xz=inicial(1,j);
       yz=inicial(2,j);
       zz=inicial(3,j);
       p(:,j)=[xz yz zz]';
       z(:,j)=p(:,j)/norm(p(:,j)');
       x(2:3,j)=x(2:3,j-1);
       x(1,j)=(-z(2,j)*x(2,j)-z(3,j)*x(3,j))/z(1,j);
       x(:,j) = x(:,j)/norm(x(:,j));
       y(:,j)=cross(z(:,j),x(:,j));
       f=[x(:,j) y(:,j) z(:,j) p(:,j)];
       Tin{j}=[f; 0 0 0 1];
    
    else    

        inicial(1,j)=(j-(intervalos))*(T1(1,4)-pmedio(1))/intervalos + pmedio(1);
        inicial(2,j)=(j-(intervalos))*(T1(2,4)-pmedio(2))/intervalos + pmedio(2);
        inicial(3,j)=(j-(intervalos))*(T1(3,4)-pmedio(3))/intervalos + pmedio(3);

       %Ejes tramo inicial
       xz=inicial(1,j);
       yz=inicial(2,j);
       zz=inicial(3,j);
       p(:,j)=[xz yz zz]';
       z(:,j)=p(:,j)/norm(p(:,j)');
       x(2:3,j)=x(2:3,j-1);
       x(1,j)=(-z(2,j)*x(2,j)-z(3,j)*x(3,j))/z(1,j);
       x(:,j) = x(:,j)/norm(x(:,j));
       y(:,j)=cross(z(:,j),x(:,j));
       f=[x(:,j) y(:,j) z(:,j) p(:,j)];
       Tin{j}=[f; 0 0 0 1];
    end
 end
 Tin{j+1}=T1;
%     figure
%     hold on
%     z=Tinicial(1:3,4);
%     v=T1(1:3,4);
%     h(:,1)=Tinicial(1:3,4);
%     for i=2:length(Tin);
%         h(:,i)=Tin{i-1}(1:3,4);
%     end
%    % h(:,7)=T1(1:3,4)
%      %plot3(z(1),z(2),z(3),'*')
%      plot3(pmedio(1),pmedio(2),pmedio(3),'*')
%      hold on
%      plot3(h(1,:),h(2,:),h(3,:)) 
%      d=length(h);
%      plot3(v(1),v(2),v(3),'*')  
end