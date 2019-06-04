%	Jose Albarranch Martinez
%	
%	CINEMATICA DE ROBOTS
%
%	PRACTICA 1 - simulación de robots de tipo serie en ARTE  I	- Cinematica inversa
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%function [q,Vtest] = inversekinematik_irb2600(robot, T)
function [q] = inversekinematic_irb2600(robot, T)
    q=zeros(6,8);
%Extraccion de parametros
    T=round(T,5);   
    theta = eval(robot.DH.theta);
    d = eval(robot.DH.d);
    a = eval(robot.DH.a);
    alpha = eval(robot.DH.alpha);
    
    L1=d(1); %0.445
    L2=a(2); %0.7
    L3=a(3); %0.115
    L6=d(6); %0.085
    A1=a(1); %0.15
    A2=d(4); %0.795
    
 % recuento de fallos
 
    fail=[0 0 0 0 0 0 0 0];%Q2
    fail2=[0 0 0 0 0 0 0 0];
    fr=[0 0 0 0 0 0 0 0];
    
  syms q1 q2 q3 q4 q5 q6;
% q1=0;q2=0;q3=0;q4=0;q5=0;q6=0;
 
 
    M1= [cos(q1),0,-sin(q1),A1*cos(q1);
        sin(q1),0,cos(q1),A1*sin(q1);
        0,-1,0,L1;
        0,0,0,1];

    M2= [sin(q2),cos(q2),0,L2*sin(q2);
        -cos(q2),sin(q2),0,L2*(-cos(q2));
         0,0,1,0;
         0,0,0,1]; 
               
    M3= [cos(q3),0,-sin(q3),L3*cos(q3);
        sin(q3),0,cos(q3),L3*sin(q3);
        0,-1,0,0;
        0,0,0,1];
    
    M4= [cos(q4),0,sin(q4),0;
        sin(q4),0,-cos(q4),0;
        0,1,0,A2;
        0,0,0,1];
    
    M5= [cos(q5),0,-sin(q5),0;
        sin(q5),0,cos(q5),0;
        0,-1,0,0;
        0,0,0,1];
    
    M6= [cos(q6+pi),-sin(q6+pi),0,0;
        sin(q6+pi),cos(q6+pi),0,0;
        0,0,1,L6;
        0,0,0,1];
   
        %M3 para el calculo de las 3 primeras
        
        b=pi/2-atan(L3/A2); 
        L23=sqrt((L3^2)+(A2^2));
        
     M3_13= [cos(q3+b),-sin(q3+b),0,L23*cos(q3+b);
         sin(q3+b),cos(q3+b),0,L23*sin(q3+b);
         0,0,1,0;
         0,0,0,1];

M03=M1*M2*M3_13;

%Efecto final

    px=T(1,4);
    py=T(2,4);
    pz=T(3,4);

%Calculo posición muñeca
  

  Pmx=px-L6*T(1,3);
  Pmy=py-L6*T(2,3);
  Pmz=pz-L6*T(3,3);
 Pm_xy=((Pmx^2 + Pmy^2)^0.5);
 sigma=[1 -1];
 
%Q2 

 for i=1:2   
     
    sig=sigma(i);

    A=-2*L2*(L1-Pmz)+ (Pm_xy^2 -2*sig*Pm_xy*A1 + A1^2+Pmz^2-2*Pmz*L1+L1^2+L2^2 - L23^2);
    B=4*L2*(A1-Pm_xy*sig);
    C=2*L2*(L1-Pmz) + (Pm_xy^2 -2*Pm_xy*sig*A1 + A1^2+Pmz^2-2*Pmz*L1+L1^2+L2^2 - L23^2);
    
    po=[A B C];
    t=roots(po);
    
    q21=(2*atan(t(1,1))); q22=(2*atan(t(2,1)));
    
  
    
    if sig==1
        q(2,1)=q21;    q(2,2)=q21;    q(2,3)=q22;    q(2,4)=q22;
    end
        q(2,5)=q21;    q(2,6)=q21;    q(2,7)=q22;    q(2,8)=q22;
        

    
    for i=1:8
        
        if ~isreal(q(2,i));  %Se aprovecha el bucle par varificar q2
            disp('WARNING:Punto no alcanzable, soluciones imaginarias');  %comprobacion y aviso
            q(2,i) = real(q(2,i));
            posicion_imaginaria=i;
            fail(posicion_imaginaria)=1;
        end
    
        sig=1;
        
        if i>4
            sig=-1;
        end
%Q3
        
%         Pm_xy*sig - A1 - L2*sin(q2) =  L23*sin(b + q3 + q2)
%         Pmz - L1 - L2*cos(q2) = L23*cos(b + q3 + q2)
%      
%         sin(b + q3 + q2)=((Pm_xy*sig - A1 - L2*sin(q2))/L23)
%         cos(b + q3 + q2)=(Pmz - L1 - L2*cos(q2))/L23
     
     seno3=((Pm_xy*sig - A1 - L2*sin(q(2,i)))/L23);
     coseno3=(Pmz - L1 - L2*cos(q(2,i)))/L23;
     
     q(3,i)=normalize(atan2(seno3,coseno3)-q(2,i)-b);
     
        if ~isreal(q(3,i));
            disp('WARNING:Punto no alcanzable, soluciones imaginarias');  %comprobacion y aviso
            q(3,i) = real(q(3,i));
            fail2(i)=1;
        end
                
%Q1
     
     coseno1=Pmx/((A1 + L2*sin(q(2,i)) + L23*cos(b + q(3,i))*sin(q(2,i)) + L23*sin(b + q(3,i))*cos(q(2,i))));
     seno1=Pmy/((A1 + L2*sin(q(2,i)) + L23*cos(b + q(3,i))*sin(q(2,i)) + L23*sin(b + q(3,i))*cos(q(2,i)))) ;  
     q(1,i)=normalize(atan2(seno1,coseno1));
%      q(1,i) = 2*atan(seno1/(1+coseno1)); 
          
    end
    
%Q5
    
    for j=1:8
        q1=q(1,j);
        q2=q(2,j);
        q3=q(3,j);
            eval(M1);
            eval(M2);
            eval(M3);
        
       M=M1*M2*M3;
       M=M';
       
       MT=eval(M*T);
        cos5=MT(3,3);
        cos4=MT(1,3);sen4=MT(2,3);
        cos6=MT(3,1);sen6=MT(3,2);
       M46=M4*M5*M6;
       
      q(5,j)=acos(cos5);
              
            if (-1)^j>0;
                q(5,j)=-q(5,j);              
            end
            
%Q4        
        coseno4=-cos4/sin(q(5,j));
        seno4=-sen4/sin(q(5,j)) ;
           q(4,j)=atan2(seno4,coseno4);
%             q(4,j) = 2*atan(seno4/(1+coseno4));
            
            if isnan(q(4,j))
                q(4,j)=0;
            end
            
%Q6            
        coseno6=-cos6/sin(q(5,j));
        seno6=sen6/sin(q(5,j)) ;
            q(6,j)=atan2(seno6,coseno6);
%             q(6,j) = 2*atan(seno6/(1+coseno6));
            if isnan(q(6,j))
                q(6,j)=0;
            end            
    end
    
    %Comprobacion de limites articulares e Imaginarios
    for m=1:8
        if or(q(1,i)<deg2rad(-180) , q(1,i)>deg2rad(180))
            fr(m)=1;
            disp('WARNING: Angulo q1 no permitido');
        end
        if or(q(2,i)<deg2rad(-95) , q(2,i)>deg2rad(155))
            fr(m)=1;
            disp('WARNING: Angulo q2 no permitido');
        end
        if or(q(3,i)<deg2rad(-180) , q(3,i)>deg2rad(75))
            fr(m)=1;
            disp('WARNING: Angulo q3 no permitido');
        end
        if or(q(5,i)<deg2rad(-120) , q(5,i)>deg2rad(120))
            %fr(m)=1;
            disp('WARNING: Angulo q5 no permitido');
        end
    end
   
   Vtest = 0;
   Test =fail + fail2 + fr;  %Si es 0, el conjunto no presenta imaginarios ni angulos fuera del rango permitido.
   for i=1:8
       if Test(i)==0
           Vtest=1;
           disp('Solucion valida');
          % break;
       end
   end
  
 end
 