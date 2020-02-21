function t=inicialfinal(y,T,robot)
tam=length(T);
T1=y*T{1};
Tf=y*T{tam};
intervalos=5;

Tinicial=directkinematic(robot,[0 0 0 0 0 0]);
 x=Tinicial(1:3,1);
 y=Tinicial(1:3,2);
 z=Tinicial(1:3,3);
 Tin{1}=Tinicial;
 
 
 for j=2:1:intervalos;
    inicial(1,j)=(j-1)*(T1(1)-Tinicial(1))/intervalos + Tinicial(1);
    inicial(2,j)=(j-1)*(T1(2)-Tinicial(2))/intervalos + Tinicial(2);
    inicial(3,j)=(j-1)*(T1(3)-Tinicial(3))/intervalos + Tinicial(3);
    
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
 x=Tf(1:3,1);
 y=Tf(1:3,2);
 z=Tf(1:3,3);
 Tfin{1}=Tf;
 
 for j=2:1:intervalos;
    final(1,j)=(j-1)*(Tinicial(1)-Tf(1))/intervalos + Tf(1);
    final(2,j)=(j-1)*(Tinicial(2)-Tf(2))/intervalos + Tf(2);
    final(3,j)=(j-1)*(Tinicial(3)-Tf(3))/intervalos + Tf(3);
   
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

 for i=1:intervalos;
    t{i}=Tin{i}; 
 end
for i=intervalos+1:tam+intervalos;
    i;
    t{i}=T{i-intervalos}; 
 end
 for i=intervalos+tam+1:tam+intervalos+intervalos;
    t{i}=Tfin{i-intervalos-tam}; 
 end
end