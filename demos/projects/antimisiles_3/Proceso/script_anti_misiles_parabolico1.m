fprintf('Coordenadas del misil objetivo\n');
mx=input('Mx: ');
my=input('My: ');
mz=input('Mz: ');
fprintf('Velocidad del antimisiles\n');
v=input('v: ')
%Ángulos del misil objetivo
beta = atan2(my,mx);
%gamma= atan2((mz-2.100017631),mx);

fprintf('MÉTODO NÚMERICO\n'); 			%TITULO

syms f(gamma) %crea una variable simbolica
f(gamma)=(mz-2.100017631)+(9.8/(2*v^2*(cos(gamma))^2))*mx^2+mx*tan(gamma);   % se ingresara la funcion

r0=0;
r1=pi/4;
i=0;
n=500;
error=0.1;
raiz=pi/2;
fprintf('Iteraciones\t\t\tr0 \t\t\tr1\t\t\t\t\t\t\t\t\t\t\t\tError\t\t\t\tRaiz\n');    	%el encabezado a imprimir
fprintf('%i\t\t\t\t%4.5f %4.5f\t\t\t\t\t\t\t\t%4.5f\\t\t\t\tn',i,r0,r1,error,raiz); %imprime nuestros datos inciales
while error>0.001
       i=i+1;
       raiz=f(r1);
            if raiz>50000
                    r1=r1-0.1;
            end;
            if raiz<-50000
                    r1=r1+0.1;
            end;
             if raiz>5000 & raiz<50000
                    r1=r1-0.1;
            end;
            if raiz<-5000 & raiz>-50000
                    r1=r1+0.1;
            end;
             if raiz>500 & raiz<5000
                    r1=r1-0.001;
            end;
            if raiz<-500 & raiz>-5000
                    r1=r1+0.001;
            end;
             if raiz>50 & raiz<500
                    r1=r1-0.0001;
            end;
            if raiz<-50 & raiz>-500
                    r1=r1+0.0001;
            end;
             if raiz>5 & raiz<50
                    r1=r1-0.00001;
            end;
            if raiz<-5 & raiz>-50
                    r1=r1+0.00001;
            end;
            if raiz>1 & raiz<5
                    r1=r1-0.000001;
            end;
            if raiz<-1 & raiz>-5
                    r1=r1+0.000001;
            end;
            if raiz>0.01 & raiz<1
                    r1=r1-0.0000001;
            end;
            if raiz<-0.01 & raiz>-1
                    r1=r1+0.0000001;
            end;
              if raiz>0 & raiz<0.01
                    r1=r1-0.00000001;
            end;
            if raiz<0 & raiz>-0.01
                    r1=r1+0.00000001;
            end;
            
            error=raiz;
            fprintf('%i\t\t\t\t%4.5f %4.5f\t\t\t\t\t\t\t\t%4.5f\t\t\t\t%4.5f\n',i,r0,abs(r1),error,raiz); %se imprimen los valores hallados por el metodo numerico
       end;
       r1=abs(r1);
       
       fprintf('Calculando velocidades y tiempo en alcanzar el objetivo...\n');
       vox=v*cos(r1);
       t=mx/vox;
       
       voy=v*sin(r1);
       fprintf('Vox=%4.5f m/s\nVoy=%4.5f m/s\n',vox,voy);
       fprintf('Tiempo=%4.5f s\n',t);     