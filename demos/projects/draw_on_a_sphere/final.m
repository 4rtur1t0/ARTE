function Tfin=final(yy,T,robot)
tam=length(T);
Tf=yy*T{tam};
global radio;
global Tinicial;
global intervalos;
%Tinicial=directkinematic(robot,[0 0 0 0 0 0]);

 x=Tf(1:3,1);
 y=Tf(1:3,2);
 z=Tf(1:3,3);
 Tfin{1}=Tf;
 pmedio=[Tf(1,4),Tf(2,4),(Tf(3,4)- radio)/2];
 
 for j=2:1:(intervalos*2);
    if j<=intervalos 
       final(1,j)=(j-1)*(pmedio(1)-Tf(1,4))/intervalos + Tf(1,4);
       final(2,j)=(j-1)*(pmedio(2)-Tf(2,4))/intervalos + Tf(2,4);
       final(3,j)=(j-1)*(pmedio(3)-Tf(3,4))/intervalos + Tf(3,4);
       %Ejes tramo final
       xz=final(1,j);
       yz=final(2,j);
       zz=final(3,j);
       p(:,j)=[xz yz zz]';
       z(:,j)=p(:,j)/norm(p(:,j)');
       x(2:3,j)=x(2:3,j-1);
       x(1,j)=(-z(2,j)*x(2,j)-z(3,j)*x(3,j))/z(1,j);
       x(:,j) = x(:,j)/norm(x(:,j));
       y(:,j)=cross(z(:,j),x(:,j));
       f=[x(:,j) y(:,j) z(:,j) p(:,j)];
       Tfin{j}=[f; 0 0 0 1];
    else
        
       final(1,j)=(j-(intervalos))*(Tinicial(1,4)-pmedio(1))/intervalos + pmedio(1);
       final(2,j)=(j-(intervalos))*(Tinicial(2,4)-pmedio(2))/intervalos + pmedio(2);
       final(3,j)=(j-(intervalos))*(Tinicial(3,4)-pmedio(3))/intervalos + pmedio(3);


       %Ejes tramo final
       xz=final(1,j);
       yz=final(2,j);
       zz=final(3,j);
       p(:,j)=[xz yz zz]';
       z(:,j)=p(:,j)/norm(p(:,j)');
       x(2:3,j)=x(2:3,j-1);
       x(1,j)=(-z(2,j)*x(2,j)-z(3,j)*x(3,j))/z(1,j);
       x(:,j) = x(:,j)/norm(x(:,j));
       y(:,j)=cross(z(:,j),x(:,j));
       f=[x(:,j) y(:,j) z(:,j) p(:,j)];
       Tfin{j}=[f; 0 0 0 1];
    end
 end
 
 Tfin{j+1}=Tinicial;
%     figure
%     hold on
%     z=Tf(1:3,4);
%     v=Tinicial(1:3,4);
%     h(:,1)=Tf(1:3,4);
%     for i=2:length(Tfin);
%         h(:,i)=Tfin{i-1}(1:3,4);
%     end
%    % h(:,7)=T1(1:3,4)
%      plot3(z(1),z(2),z(3),'*')
%      plot3(pmedio(1),pmedio(2),pmedio(3),'*')
%      hold on
%      plot3(h(1,:),h(2,:),h(3,:)) 
%      d=length(h);
%      plot3(v(1),v(2),v(3),'*') 
end