% Resolucion laberinto regla mano derecha
% 1 es el camino que puede seguir el robot
% 3 son las salidas
% 0 es hueco en blanco que no puede seguir el robot

MAP = [3,0,0,1,1,1,3,0,0,0,0,0,0,0,0,0,0,0,0,1,3,0,0,3,0,0;
       1,0,0,1,0,0,0,0,3,0,0,0,0,1,1,1,3,0,0,1,0,0,0,1,0,0;
       1,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0;
       1,0,0,1,0,0,0,0,1,1,1,1,0,1,0,0,0,1,1,1,1,1,1,1,0,0;
       1,0,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0;
       1,1,1,1,1,1,1,3,0,0,0,1,0,1,1,1,1,1,1,3,0,0,0,1,0,0;
       1,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0;
       1,0,0,1,0,0,3,0,0,0,0,1,0,0,0,0,0,3,0,0,0,0,0,1,0,0;
       1,0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0;
       1,0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,1,1,0,0;
       3,0,0,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,0,0;
       0,0,0,1,0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,0;
       0,0,0,1,0,0,1,0,0,0,3,1,1,1,0,0,0,0,1,0,0,1,0,1,0,0;
       0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0;
       1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,0,1,0,3,0,0;
       1,0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0;
       1,0,0,1,0,0,0,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,1,1,0,0;
       1,0,0,1,0,0,0,1,0,0,1,0,0,0,0,1,0,1,3,0,0,0,0,1,0,0;
       3,0,0,3,0,0,0,3,0,0,1,1,1,1,1,1,0,1,0,0,0,0,0,1,0,0;
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0;
       1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0;
       1,0,0,3,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0;
       1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,3,0,0,0,0,0,1,0,0;
       1,1,1,1,1,1,1,1,0,0,3,1,1,1,1,1,1,1,0,0,0,0,0,3,0,0;
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
   n=26;
   MAProuta =0*ones(n,n);
   MAPsinedit=MAP;
   startX=1;
   startY=1;
   x=startX;
   y=startY;
  	%rout=ruta(MAP,2,4)
  rutaencontrada = 0;
     MAProuta(x,y) = 2; % 7 CAMINO DEL ROBOT
    while rutaencontrada == 0
       % ESQUINAS
       if y-1<1 && x-1<1 
            if MAP(x,y+1) == 3 
                 MAProuta(x,y+1) = 3;
                 y=y+1;
                 rutaencontrada=1;
             elseif MAP(x+1,y) == 3
                 MAProuta(x+1,y) = 3;
                 x=x+1;
                 rutaencontrada=1;
                 
                 %4 CASOS
             elseif MAP(x,y+1) == 1 
                 MAProuta(x,y+1) = 1;
                  MAP(x,y) = 0;
                 y=y+1;
             elseif MAP(x+1,y) == 1
                 MAProuta(x+1,y) = 1;
                    MAP(x,y) = 0;
                 x=x+1;
             else
                rutaencontrada=1;
                
             end
           
       elseif y-1<1 && x+1>n 
            if MAP(x,y+1) == 3 
                 MAProuta(x,y+1) = 3;
                 y=y+1;
                 rutaencontrada=1;
    
             elseif MAP(x-1,y) == 3
                 MAProuta(x-1,y) = 3; % SALIDA 6
                 x=x-1;
                 rutaencontrada=1;
                 
                 %4 CASOS
             elseif MAP(x,y+1) == 1 
                 MAProuta(x,y+1) = 1;
                    MAP(x,y) = 0;
                 y=y+1;
             elseif MAP(x-1,y) == 1
                 MAProuta(x-1,y) = 1;
                    MAP(x,y) = 0;
                 x=x-1;
             else
                rutaencontrada=1;
                
             end
       elseif y+1>n && x+1>n 
           
             if MAP(x,y-1) == 3
                 MAProuta(x,y-1) = 3;
                 y=y-1;
                 rutaencontrada=1;
             elseif MAP(x-1,y) == 3
                 MAProuta(x-1,y) = 3; % SALIDA 6
                 x=x-1;
                 rutaencontrada=1;
                 
                 %4 CASOS
          
             elseif MAP(x,y-1) == 1
                 MAProuta(x,y-1) = 1;
                    MAP(x,y) = 0;
                 y=y-1;
             elseif MAP(x-1,y) == 1
                 MAProuta(x-1,y) = 1;
                    MAP(x,y) = 0;
                 x=x-1;
             else
                rutaencontrada=1;
                
             end
       elseif y+1>n && x-1<1 
        
             if MAP(x+1,y) == 3
                 MAProuta(x+1,y) = 3;
                 x=x+1;
                 rutaencontrada=1;
             elseif MAP(x,y-1) == 3
                 MAProuta(x,y-1) = 3;
                 y=y-1;
                 rutaencontrada=1;
           
                 
                 %4 CASOS
             
             elseif MAP(x+1,y) == 1
                 MAProuta(x+1,y) = 1;
                    MAP(x,y) = 0;
                 x=x+1;
             elseif MAP(x,y-1) == 1
                 MAProuta(x,y-1) = 1;
                    MAP(x,y) = 0;
                 y=y-1;
             else
                rutaencontrada=1;
                
             end
           % FIN ESQUINAS
           % COMIENZO BORDES
       elseif y-1<1
                if MAP(x,y+1) == 3 
                 MAProuta(x,y+1) = 3;
                 y=y+1;
                 rutaencontrada=1;
             elseif MAP(x+1,y) == 3
                 MAProuta(x+1,y) = 3;
                 x=x+1;
                 rutaencontrada=1;
             elseif MAP(x-1,y) == 3
                 MAProuta(x-1,y) = 3; % SALIDA 6
                 x=x-1;
                 rutaencontrada=1;
                 
                 %4 CASOS
             elseif MAP(x,y+1) == 1 
                 MAProuta(x,y+1) = 1;
                    MAP(x,y) = 0;
                 y=y+1;
             elseif MAP(x+1,y) == 1
                 MAProuta(x+1,y) = 1;
                    MAP(x,y) = 0;
                 x=x+1;
             elseif MAP(x-1,y) == 1
                 MAProuta(x-1,y) = 1;
                    MAP(x,y) = 0;
                 x=x-1;
             else
                rutaencontrada=1;
                end
                
    elseif y+1>n
        
                 if MAP(x+1,y) == 3
                 MAProuta(x+1,y) = 3;
                 x=x+1;
                 rutaencontrada=1;
             elseif MAP(x,y-1) == 3
                 MAProuta(x,y-1) = 3;
                 y=y-1;
                 rutaencontrada=1;
             elseif MAP(x-1,y) == 3
                 MAProuta(x-1,y) = 3; % SALIDA 6
                 x=x-1;
                 rutaencontrada=1;
                 
                 %4 CASOS
             elseif MAP(x+1,y) == 1
                 MAProuta(x+1,y) = 1;
                    MAP(x,y) = 0;
                 x=x+1;
             elseif MAP(x,y-1) == 1
                 MAProuta(x,y-1) = 1;
                    MAP(x,y) = 0;
                 y=y-1;
             elseif MAP(x-1,y) == 1
                 MAProuta(x-1,y) = 1;
                    MAP(x,y) = 0;
                 x=x-1;
             else
                rutaencontrada=1;
                 end
                 
     elseif x-1<1
                  if MAP(x,y+1) == 3 
                 MAProuta(x,y+1) = 3;
                 y=y+1;
                 rutaencontrada=1;
             elseif MAP(x+1,y) == 3
                 MAProuta(x+1,y) = 3;
                 x=x+1;
                 rutaencontrada=1;
             elseif MAP(x,y-1) == 3
                 MAProuta(x,y-1) = 3;
                 y=y-1;
                 rutaencontrada=1;
           
                 
                 %4 CASOS
             elseif MAP(x,y+1) == 1 
                 MAProuta(x,y+1) = 1;
                    MAP(x,y) = 0;
                 y=y+1;
             elseif MAP(x+1,y) == 1
                 MAProuta(x+1,y) = 1;
                    MAP(x,y) = 0;
                 x=x+1;
             elseif MAP(x,y-1) == 1
                 MAProuta(x,y-1) = 1;
                    MAP(x,y) = 0;
                 y=y-1;
             else
                rutaencontrada=1;
                  end
                  
     elseif x+1>n
         
                if MAP(x,y+1) == 3 
                 MAProuta(x,y+1) = 3;
                 y=y+1;
                 rutaencontrada=1;
             elseif MAP(x,y-1) == 3
                 MAProuta(x,y-1) = 3;
                 y=y-1;
                 rutaencontrada=1;
             elseif MAP(x-1,y) == 3
                 MAProuta(x-1,y) = 3; % SALIDA 6
                 x=x-1;
                 rutaencontrada=1;
                 
                 %4 CASOS
             elseif MAP(x,y+1) == 1 
                 MAProuta(x,y+1) = 1;
                    MAP(x,y) = 0;
                 y=y+1;
             elseif MAP(x,y-1) == 1
                 MAProuta(x,y-1) = 1;
                    MAP(x,y) = 0;
                 y=y-1;
             elseif MAP(x-1,y) == 1
                 MAProuta(x-1,y) = 1;
                    MAP(x,y) = 0;
                 x=x-1;
             else
                rutaencontrada=1;
                end
                % FIN BORDES
         else
             if MAP(x,y+1) == 3 
                 MAProuta(x,y+1) = 3;
                 y=y+1;
                 rutaencontrada=1;
             elseif MAP(x+1,y) == 3
                 MAProuta(x+1,y) = 3;
                 x=x+1;
                 rutaencontrada=1;
             elseif MAP(x,y-1) == 3
                 MAProuta(x,y-1) = 3;
                 y=y-1;
                 rutaencontrada=1;
             elseif MAP(x-1,y) == 3
                 MAProuta(x-1,y) = 3; % SALIDA 6
                 x=x-1;
                 rutaencontrada=1;
                 
                 %4 CASOS
             elseif MAP(x,y+1) == 1 
                 MAProuta(x,y+1) = 1;
                    MAP(x,y) = 0;
                 y=y+1;
             elseif MAP(x+1,y) == 1
                 MAProuta(x+1,y) = 1;
                    MAP(x,y) = 0;
                 x=x+1;
             elseif MAP(x,y-1) == 1
                 MAProuta(x,y-1) = 1;
                    MAP(x,y) = 0;
                 y=y-1;
             elseif MAP(x-1,y) == 1
                 MAProuta(x-1,y) = 1;
                    MAP(x,y) = 0;
                 x=x-1;
             else
                rutaencontrada=1;
                
             end
        end
    end
    figure;
    imagesc(MAProuta)
    caxis([0 3]);
    figure;
    imagesc(MAPsinedit)
   caxis([0 3]);
    
    
    
        