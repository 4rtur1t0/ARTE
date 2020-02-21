function T=paint_robot
puntos=path_planning;
%robot=load_robot('KUKA','kr160_r1570_nanoC');
l=size(puntos);
longitud=l(2);
   xz=puntos(1,1);
   yz=puntos(2,1);
   zz=puntos(3,1);
   p(:,1)=[xz yz zz]';
   z(:,1)=p(:,1)/norm(p(:,1));
   c=null(z(:,1)');
   x(:,1)=c(:,1);
   y(:,1)=c(:,2);
   i=1;
   f=[x(:,i) y(:,i) z(:,i) p(:,i)];
   T{i}=[f; 0 0 0 1];
   %T(1,:,:)=[x(:,1) y(:,1) z(:,1) p(:,1)]';
   
for i=2:1:longitud;
   xz=puntos(1,i);
   yz=puntos(2,i);
   zz=puntos(3,i);
   p(:,i)=[xz yz zz]';
   z(:,i)=p(:,i)/norm(p(:,i)');
   x(2:3,i)=x(2:3,i-1);
   x(1,i)=(-z(2,i)*x(2,i)-z(3,i)*x(3,i))/z(1,i);
   x(:,i) = x(:,i)/norm(x(:,i));
   y(:,i)=cross(z(:,i),x(:,i));
   %p(4,i)
   f=[x(:,i) y(:,i) z(:,i) p(:,i)];
   T{i}=[f; 0 0 0 1];
   %T(i,:,:)=[x(:,i) y(:,i) z(:,i) p(:,i)]';
end
   



end 