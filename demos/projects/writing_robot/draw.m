function puntos=draw(frase)
%cd 'C:\Program Files\MATLAB\R2017b\bin\arte';

load hershey
global robot
init_lib;
robot=load_robot('kuka', 'KR180_R2100');

% origen=input('Nuevo origen:');
% alpha=input('Ángulo de giro en OX (grad):');
% beta=input('Ángulo de giro en OY (grad):');
% gamma=input('Ángulo de giro en OZ (grad):');
% escala=input('Escala:');
% velocidad=input('Velocidad (m/s):');


origen=[0.5 0.5 0.3];
alpha=10;
beta=10;
gamma=30;
escala=0.1;
velocidad=0.002;

% 25 puntos/seg * 1/velocidad seg/m = 25/velocidad (puntos/metro)
% puntos_metro=10/velocidad;
puntos_metro=100;

Rx=[1 0 0 0; 0 cosd(alpha) -sind(alpha) 0; 0 sind(alpha) cosd(alpha) 0; 0 0 0 1];
Ry=[cosd(beta) 0 sind(beta) 0; 0 1 0 0; -sind(beta) 0 cosd(beta) 0; 0 0 0 1];
Rz=[cosd(gamma) -sind(gamma) 0 0; sind(gamma) cosd(gamma) 0 0; 0 0 1 0; 0 0 0 1];
trans=eye(4);
trans(1:3,4)=origen';
%Hallo la metriz T para poder trasladar y rotar mis letras
T=Rx*Ry*Rz*trans;
T(4,4)=escala;

j=1;
t=-1;
espacio=0;

    for i=0:(size(frase,2)-1)   
        if frase(i+1)==' ' 
            %Si hay un espacio, le sumo 0,8 a X
            if t>0   
                espacio=0.8;
            end  
        else
            %Este bucle me detecta si hay una o más letras en la frase y me
            %saca los puntos de la frase
            letra = hershey{frase(i+1)};
            letra.stroke;                                                         

            path = [0.25*letra.stroke; zeros(1,size(letra.stroke,2))];               
            k = find(isnan(path(1,:)));                                      
            path(:,k) = path(:,k-1);                                          
            path(3,k) = 0.2;

            path(1,:) = path(1,:)+0.3*i + espacio;

            j=size(path,2);

            aux(1,t+2:t+j+1)=path(1,:);
            aux(2,t+2:t+j+1)=path(2,:);
            aux(3,t+2:t+j+1)=path(3,:);      
            if t>0      
                aux(1,t+1)=aux(1,t);
                aux(2,t+1)=aux(2,t);
                aux(3,t+1)=aux(3,t)+0.2;
            end
        end
        t=size(aux,2);    
        m=0;
        espacio=0;
    end
   
   muestreado=[0]; 
   
    for u=1:size(aux,2)
        
%         puntos_por_segmento=(xmax-xmin)*puntos_metro
%         paso=(xmax-xmin)/puntos_por_segmento

        if u==size(aux,2)
            t=size(muestreado,2);
            muestreado(1,t)=aux(1,u);
            muestreado(2,t)=aux(2,u);
            muestreado(3,t)=aux(3,u);
        else
            director=aux(:,u+1)-aux(:,u); %Me da el vector director
            if director(1,1)~=0
                if aux(1,u+1)>=aux(1,u)
                    signo=1;
                else
                    signo=-1;
                end
                for x=aux(1,u) : signo/puntos_metro : aux(1,u+1)
                    landa=(x-aux(1,u))/director(1,1);
                    y=aux(2,u)+landa*director(2,1);
                    z=aux(3,u)+landa*director(3,1);
                   if x==aux(1,u)
                        if size(muestreado,2)==1
                            t=size(muestreado,2);
                            muestreado(1,t)=aux(1,u);
                            muestreado(2,t)=aux(2,u);
                            muestreado(3,t)=aux(3,u);
                        else
                            t=size(muestreado,2);
                            muestreado(1,t+1)=x;
                            muestreado(2,t+1)=y;
                            muestreado(3,t+1)=z;
                        end
                   else
                        t=size(muestreado,2);
                        muestreado(1,t+1)=x;
                        muestreado(2,t+1)=y;
                        muestreado(3,t+1)=z;
                   end
                end
            else
                if director(2,1)~=0
                    if aux(2,u+1)>=aux(2,u)
                        signo=1;
                    else
                        signo=-1;
                    end
                    for y=aux(2,u) : signo/puntos_metro : aux(2,u+1)
                        landa=(y-aux(2,u))/director(2,1);
                        x=aux(1,u);
                        z=aux(3,u)+landa*director(3,1);
                        if y==aux(2,u)
                            if size(muestreado,2)==1
                                t=size(muestreado,2);
                                muestreado(1,t)=aux(1,u);
                                muestreado(2,t)=aux(2,u);
                                muestreado(3,t)=aux(3,u);
                            else
                                t=size(muestreado,2);
                                muestreado(1,t+1)=x;
                                muestreado(2,t+1)=y;
                                muestreado(3,t+1)=z;
                            end
                        else
                            t=size(muestreado,2);
                            muestreado(1,t+1)=x;
                            muestreado(2,t+1)=y;
                            muestreado(3,t+1)=z;
                        end
                    end
                else
                    if director(3,1)~=0
                        if aux(3,u+1)>=aux(3,u)
                            signo=1;
                        else
                            signo=-1;
                        end
                        for z=aux(3,u) : signo/puntos_metro : aux(3,u+1)
                            landa=(z-aux(3,u))/director(3,1);
                            x=aux(1,u);
                            y=aux(2,u);
                            if z==aux(3,u)
                                if size(muestreado,2)==1
                                    t=size(muestreado,2);
                                    muestreado(1,t)=aux(1,u);
                                    muestreado(2,t)=aux(2,u);
                                    muestreado(3,t)=aux(3,u);
                                else
                                    t=size(muestreado,2);
                                    muestreado(1,t+1)=x;
                                    muestreado(2,t+1)=y;
                                    muestreado(3,t+1)=z;
                                end
                            else
                                t=size(muestreado,2);
                                muestreado(1,t+1)=x;
                                muestreado(2,t+1)=y;
                                muestreado(3,t+1)=z;                       
                            end
                        end
                    end
                end
            end
        end
    end
    
    muestreado(4,:)=1;
   
    for m=1:size(muestreado,2)
        %Este bucle  me multiplica los puntos hallados antes en el SCP
        %original y me los pasa a mi SCP (guardado en 'puntos')
        
        puntos(1:4,m)=T*muestreado(1:4,m);
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    primer_punto=directkinematic(robot,[0 0 0 0 0 0]);

    mn=1;
    nm=1;
    
    if primer_punto(1,4)>puntos(1,1)
        signo1=-1;
    else
        signo1=1;
    end
    
    for x1=primer_punto(1,4) : signo1/puntos_metro : puntos(1,1)
        director1=puntos(1:3,1)-primer_punto(1:3,4);
        landa=(x1-primer_punto(1,4))/director1(1,1);
        y1=primer_punto(2,4)+landa*director1(2,1);
        z1=primer_punto(3,4)+landa*director1(3,1);
        inicial(1:4,mn)=[x1; y1; z1; 1];
        mn=mn+1;
    end
    
    if primer_punto(1,4)>puntos(1,size(puntos,2))
        signo2=1;
    else
        signo2=-1;
    end    
    
    for x2=puntos(1,size(puntos,2)) : signo2/puntos_metro : primer_punto(1,4)
        director2=puntos(1:3,size(puntos,2))-primer_punto(1:3,4);
        landa=(x2-puntos(1,size(puntos,2)))/director2(1,1);
        y2=puntos(2,size(puntos,2))+landa*director2(2,1);
        z2=puntos(3,size(puntos,2))+landa*director2(3,1);
        final(1:4,nm)=[x2; y2; z2; 1];
        nm=nm+1;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
trayectoria=[inicial,puntos,final];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Obtengo el eje X, Y y Z del efector final
    efectorX=-T(1:3,1);
    efectorY=-T(1:3,2);
    efectorZ=-T(1:3,3);

    %Creo la matriz Tefector para el primer punto
    Tefector(:,1)=[efectorX;0];
    Tefector(:,2)=[efectorY;0];
    Tefector(:,3)=[efectorZ;0];
    Tefector(:,4)=[trayectoria(1:3,1);1];
        
    for w=1:size(trayectoria,2)
        tiempo=puntos_metro/velocidad;
        u=w;
        if w==1
            q1_todas=inversekinematic(robot,Tefector);
            q1=q1_todas(:,1);

            vx=(trayectoria(1,w+1)-trayectoria(1,w))/tiempo;
            vy=(trayectoria(2,w+1)-trayectoria(2,w))/tiempo;
            vz=(trayectoria(3,w+1)-trayectoria(3,w))/tiempo;

            qd = inverse_kinematic_moore_penrose(robot, q1, [vx vy vz 0 0 0]);

            q2=q1+qd*tiempo;
            q(:,w)=q1;
            q(:,w+1)=q2;
        elseif w==size(trayectoria,2)
             q(:,w+1)=q2;
        else
            vx=(trayectoria(1,w+1)-trayectoria(1,w))/tiempo;
            vy=(trayectoria(2,w+1)-trayectoria(2,w))/tiempo;
            vz=(trayectoria(3,w+1)-trayectoria(3,w))/tiempo;

            qd = inverse_kinematic_moore_penrose(robot, q2, [vx vy vz 0 0 0]);

            q2=q2+qd*tiempo; %Mod_posicion/v es el tiempo que tarda para ir de un punto a otro

            q(:,w+1)=q2;   
        end
    end
    
    
    plano=[-0.2 2 2  -0.2 -0.2; -0.3 -0.3 0.3 0.3 -0.3; 0 0 0 0 0; 1 1 1 1 1];
    for nm=1:size(plano,2)
        plano(:,nm)=T*plano(:,nm);
    end
    
    animatetraj(robot, q, trayectoria, plano);


end


function qd = inverse_kinematic_moore_penrose(robot, q, v)
J = manipulator_jacobian(robot, q);
Jp = pinv(J);
qd = Jp*v';
end

function animatetraj(robot, q, traj, plano)
global configuration 

h=figure(configuration.figure.robot); hold on,

adjust_view(robot);

plot3(plano(1,:),plano(2,:),plano(3,:),'y','LineWidth',2);


%get adjusted view
[az,el] = view;
for j=1:size(q, 2)
    qj=q(:,j);  
    view(az,el);
    % axis(robot.axis)
    %draw robot in 3D
    drawrobot3d(robot, qj); 
    
    plot3(traj(1,:),traj(2,:),traj(3,:),'r','LineWidth',2)
    plot3(plano(1,:),plano(2,:),plano(3,:),'r','LineWidth',5);
    
    %pause to get a nice view
    pause(configuration.time_delay);   
end
end

function adjust_view(robot)

if exist('robot', 'var')
    drawrobot3d(robot, robot.q);
end
fprintf('\n\nAJUSTA LA VISTA');
fprintf(2, '\nPresiona cualquier tecla para continuar... ') ;

pause
%get adjusted view
[az, el] = view;
end 