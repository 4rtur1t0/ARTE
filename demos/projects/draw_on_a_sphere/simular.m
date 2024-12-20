function tt=simular(robot,t,y,mode)

    global radio
    global precision
    
    drawrobot3d(robot,[0 0 0 0 0 0])
    axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
    adjust_view()
    f=length(t);
    d=1;

    pause(3)
    %Recorrido desde posicion inicial hasta inicio pintado
     tin=inicial(y,t,robot);
     in=length(tin);
     for i=1:in;
        T=tin{i};
        q = inversekinematic_kuka_kr160_r1570_nanoC(robot, T) ;
        drawrobot3d(robot, q)

        axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
        hold on
        

         treal=directkinematic(robot,q(:,1));   
         pin(:,i)=treal(1:3,4);
         tt{d}=treal;
         d=d+1;  

         Representacion_en_esfera(y);
         hold on
        axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
         switch mode
             case 'trayectoria'
                 plot3(pin(1,:),pin(2,:),pin(3,:));
                 
             case 'letras'
                 
         end
       % pause(0.005);
    end
    
    %Pintado esfera
    let=1;
    frase=[0; 0; 0];
    for i=1:f;
        T=y*t{i};
        q = inversekinematic_kuka_kr160_r1570_nanoC(robot, T) ;
        drawrobot3d(robot, q)

        axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
        hold on
        treal=directkinematic(robot,q(:,1));   
        tt{d}=treal;
        d=d+1;

            if abs((treal(1,4)-y(1,4))^2+(treal(2,4)-y(2,4))^2+(treal(3,4)-y(3,4))^2-radio^2) < precision
                frase(:,let)=treal(1:3,4);
                let=let+1;
            end
            p(:,i)=treal(1:3,4);
          
        %end
         Representacion_en_esfera(y);
         hold on
        axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
         switch mode
             case 'trayectoria'
                  plot3(p(1,:),p(2,:),p(3,:));
                  plot3(pin(1,:),pin(2,:),pin(3,:));
             case 'letras'
                  plot3(frase(1,:),frase(2,:),frase(3,:));
         end

       % pause(0.005);
    end
    
    %Recorrido desde posicion final del pintado hasta posicion inicial
     tfin=final(y,t,robot);
     fin=length(tfin);
     for i=1:fin;
        T=tfin{i};
        q = inversekinematic_kuka_kr160_r1570_nanoC(robot, T) ;
        drawrobot3d(robot, q)

        axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
        hold on
        treal=directkinematic(robot,q(:,1));   
        tt{d}=treal;
        d=d+1;

            pfin(:,i)=treal(1:3,4);
            %axis([-1 1 -1 1]);
           
       % end
        Representacion_en_esfera(y);
        hold on
        axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
         switch mode
             case 'trayectoria'
                  plot3(p(1,:),p(2,:),p(3,:));
                  plot3(pin(1,:),pin(2,:),pin(3,:));
                  plot3(pfin(1,:),pfin(2,:),pfin(3,:));
             case 'letras'
                 plot3(frase(1,:),frase(2,:),frase(3,:),'r');

         end

       % pause(0.005);
    end
    
    drawrobot3d(robot, [0 0 0 0 0 0])
    hold on

    Representacion_en_esfera(y);

        axis([-1.5 1.50 -1.5 0.75 -0.25 1.75]);
         switch mode
             case 'trayectoria'
                  plot3(p(1,:),p(2,:),p(3,:));
                  plot3(pin(1,:),pin(2,:),pin(3,:));
                  plot3(pfin(1,:),pfin(2,:),pfin(3,:));
             case 'letras'
                 %figure
                  %Representacion_en_esfera(y);
                 %hold on
                  plot3(frase(1,:),frase(2,:),frase(3,:),'r');
                  
         end
end
