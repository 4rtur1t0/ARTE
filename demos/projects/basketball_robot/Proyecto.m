%%%Cargamos el robot de 3DOFplanar fuera del script
%Variables predefinidas
teta=30; %Ángulo mínimo de entrada a la canasta en grados
xr=4; %Distania entre la base del robot y la canasta
teta0=60;%Ángulo de salida del balón
wmax=5; %velocidad angular máxima de los eslabones
N=10; %Número de puntos en los bucles de q1, q2 y q3
N2=20; %Número de puntos para el bucle de velocidades angulares
%Pasos de los bucles:
delta1=(pi)/(N-1); 
delta2=2*pi/(N-1);
delta3=2*wmax/(N2-1); %Paso del bucle de velocidades angulares
Q=[]; %Matriz de soluciones


for q1=0:delta1:pi %Así descartamos los valores de q1 que hacen que el eslabón 1 atraviese el suelo.
    for q2=-pi:delta2:pi
        for q3=-pi:delta2:pi
                    
            A01=dh(q1,0,1,0);       %Con estos cálculos podemos saber si los 
            A12=dh(q2,0,1,0);       %eslabones 2 y 3 atraviesan el suelo mediante
            A13=dh(q3,0,1,0);       %sus matrices de transformación
            A02=A01*A12;
            A03=A02*A13;
            p2=A02(:,4);
            p3=A03(:,4);
            if p3(2)<0 || p2(2)<0 %Si las cordenadas y de los eslabones 2 y tres son negativas
                continue %Descartamos los valores que hacen que el robot choque con el suelo
            end
            
            %Cálculo del tiro parabólico
            x=xr+p3(1); %Coordenada horizontal del punto de lanzamiento
            y=p3(2); %Coordinada vertical del punto de lanzamiento
            h=3.05-y; %Diferencia de alturas entre el punto y la canasta
            v0=sqrt(9.81*x/(2*(cos(teta0*pi/180))^2*(tan(teta0*pi/180)-h/x)));
            if v0==0
                continue
            end
            if imag(v0)~=0
                continue
            end
            tetaE=atan((2*h)/x-tan(teta0*pi/180)); %Ángulo de entrada a la canasta
            if -tetaE<(teta*pi/180);
                continue %Descartamos los valores menores al ángulo mínimo de entrada
            end
            
            for w=-wmax:delta3:wmax 
                J0=compute_jacobian(robot,[q1 q2 q3]);
                J=[J0(1,:);J0(2,:)];
                J1=J(:,1);
                J2=J(:,2);
                J3=J(:,3);
                Vx=-v0*cos(teta0*pi/180);
                Vy=v0*sin(teta0*pi/180);
                d=sqrt(x^2+h^2);
                if det([J1 J2])~=0
                    w3=w;
                    W=inv([J1 J2])*([Vx;Vy]-J3*w3);
                    w1=W(1);
                    w2=W(2);
                    if abs(w1)<=wmax && abs(w2)<=wmax
                        Q=[Q;q1 q2 q3 v0 x y d];
                        break
                    end
                end
                if det([J1 J3])~=0
                    w2=w;
                    W=inv([J1 J3])*([Vx;Vy]-J2*w2);
                    w1=W(1);
                    w3=W(2);
                    if abs(w1)<=wmax && abs(w3)<=wmax
                        Q=[Q;q1 q2 q3 v0 x y d];
                        break
                    end
                end
                if det([J2 J3])~=0
                    w1=w;
                    W=inv([J2 J3])*([Vx;Vy]-J1*w1);
                    w2=W(1);
                    w3=W(2);
                    if abs(w2)<=wmax && abs(w3)<=wmax
                        Q=[Q;q1 q2 q3 v0 x y d];
                        break
                    end
                end
            end
        end
    end
end

if isempty(Q)==1
    disp('El algoritmo no ha sido capaz de encontrar valores debido a que las condiciones iniciales no lo permiten')
else
    Q1=Q(:,1);
    Q2=Q(:,2);
    Q3=Q(:,3);
    V0=Q(:,4);
    X=Q(:,5);
    Y=Q(:,6);
    D=Q(:,7);
    [V,I]=max(D);
    disp('Posición articular:')
    q1=Q1(I)
    q2=Q2(I)
    q3=Q3(I)
    disp('Distancia')
    D=V
    Sol=[q1 q2 q3 V];
end