function [xx,yy,zz]=Multipunto_interpolacion(x,y,z)
% x=0:0.5:3;
% y=randi(10,1,7);
 b=size(x);
 bb=b(1,2);
 global multipuntos;
%Primer bucle: barrido entre todos los puntos de entrada
%Segundo bucle: generacion de la interpolacion
i=1;

for i=2:1:bb-1;
    if z(i)==0
%         if z(i-1)==0
%             for j=1:1:puntos;
% 
%                xx((i-1)*puntos+j)=(j-1)*(x(i+1)-x(i))/puntos + x(i);
%                yy((i-1)*puntos+j)=(j-1)*(y(i+1)-y(i))/puntos + y(i);
%                zz((i-1)*puntos+j)=z(i);
% 
%             end
%         else
            for j=1:1:multipuntos;

               xx((i-1)*multipuntos+j)=(j-1)*(x(i+1)-x(i))/multipuntos + x(i);
               yy((i-1)*multipuntos+j)=(j-1)*(y(i+1)-y(i))/multipuntos + y(i);
               zz((i-1)*multipuntos+j)=z(i);

            end
%         end
    else
        %Si z ~= 0, no se muestrea
        j=1;
        xx((i-1)*multipuntos+j)=xx((i-1)*multipuntos+j-1);
        yy((i-1)*multipuntos+j)= yy((i-1)*multipuntos+j-1);
        zz((i-1)*multipuntos+j)=z(i);
    end
end
% figure
% plot3(xx',yy',zz');

end

% figure
% plot(xx,yy,'*');
% hold on
% plot(x,y);