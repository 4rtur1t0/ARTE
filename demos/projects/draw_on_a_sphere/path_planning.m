function punto=path_planning
    global escala;    
    global interpolacion;
    global radio;
    
    %Obtenemos texto
    [x,y,z]=letras;  
    %f=[x;y;z]
    maxx=max(x);
    maxy=max(y);
    %Interpolacion
    %plot3(x,y,z,'*');
    %hold on
    [xxa,yya,zza]=Multipunto_interpolacion(x,y,z);
    
    d=length(xxa);
    h=1;
    for i=1:d;
        if xxa(i)==0
            continue
        else 
            auxx(h)=xxa(i);
            auxy(h)=yya(i);
            auxz(h)=zza(i);
            h=h+1;
        end
    end
    %plot3(xxa,yya,zza,'*')
    %plot3(auxx,auxy,auxz,'+')
    xxa=auxx;
    yya=auxy;
    zza=auxz;
    f=[xxa;yya;zza];
    %plot3(xxa,yya,zza,'*')
    %Centrado (respecto esfera, sino escribe sobre el "ecuador" y no a traves del polo)
    xx=xxa-maxx/2;
    yy=yya-maxy/2;
    %Escalado y union en punto
    p=[xx' yy' zza'].*escala;
    b=size(p);
    longitud=b(1);
    
    %figure
    %hold on
    %Proyeccion estereográfica
    [xyz, lin]=en_esfera(p(1,:));
    puntos(:,1)=xyz;
    k=2;
    for i=2:1:longitud;
        if isnan(p(i,1))==1
            continue
        end 
        
        if p(i-1,3)==0
            if p(i,3)==0
                [xyz, lin]=en_esfera(p(i,:));
                puntos(:,k)=xyz;
                k=k+1;
            else
                V=-[0,0,0]'+puntos(:,k-1);
                aux=puntos(:,k-1)+V.*0.2;
                
                for j=1:1:interpolacion;
                    puntos(1,k+j)=(j-1)*(aux(1)-puntos(1,k-1))/interpolacion + puntos(1,k-1);
                    puntos(2,k+j)=(j-1)*(aux(2)-puntos(2,k-1))/interpolacion + puntos(2,k-1);
                    puntos(3,k+j)=(j-1)*(aux(3)-puntos(3,k-1))/interpolacion + puntos(3,k-1);
                end
                k=k+interpolacion;
            end
        else
                p(i,:);
                i;
                [xyz, lin]=en_esfera(p(i,:));
                %plot3(lin(1,:),lin(2,:),lin(3,:),'r')
                %hold on
            for j=1:1:interpolacion;
                puntos(1,k+j)=(j-1)*(-aux(1)+xyz(1))/interpolacion + aux(1);
                puntos(2,k+j)=(j-1)*(-aux(2)+xyz(2))/interpolacion + aux(2);
                puntos(3,k+j)=(j-1)*(-aux(3)+xyz(3))/interpolacion + aux(3);
             end
            k=k+interpolacion;
            puntos(:,k)=xyz;
            k=k+1;
        end
    end
    %Validacion de los puntos (debug)
     %plot3(puntos(1,:),puntos(2,:),puntos(3,:),'*')
%     punto(1,:)=nonzeros(puntos(1,:));
%     punto(2,:)=nonzeros(puntos(2,:));
%     punto(3,:)=nonzeros(puntos(3,:));
    f=1;
    for i=1:length(puntos)
       if puntos(1,i)==0 && puntos (2,i)==0
       
       else
           punto(:,f)=puntos(:,i);
           f=f+1;
       end
    end
%     axis([-0.3 0.3 -0.3 0.3 -0.3 0.3])
  %   Representacion_en_esfera([1 0 0 0; 0 1 0 0; 0 0 1 0;0 0 0 1]);
%     hold on
 %    plot3(punto(1,:),punto(2,:),punto(3,:))

end