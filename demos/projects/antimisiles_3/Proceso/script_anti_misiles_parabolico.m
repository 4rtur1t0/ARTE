fprintf('Coordenadas del misil objetivo\n');
mx=input('Mx: ');
my=input('My: ');
mz=input('Mz: ');
fprintf('Velocidad del antimisiles\n');
v=input('v: ')
%Ángulos del misil objetivo
beta = atan2(my,mx);
%gamma= atan2((mz-2.100017631),mx);

fprintf('METODO NEWTON-RAPHSON\n'); 			%TITULO

syms f(gamma) %crea una variable simbolica
f(gamma)=(mz-2.100017631)+(9.8/(2*v^2*(cos(gamma))^2))*mx^2+mx*tan(gamma);   % se ingresara la funcion

%r0=input('ingrese la primera aproximacion: ');  	%se ingresa el primer punto o raiz
r0=0.5;
%tol=input('Ingrese la tolerancia: '); %valor del error minimo aceptable
tol=1;
%n=input('Ingrese numero maximo de iteraciones: '); 	%hasta cuantas iteraciones 
n=15;
df=diff(f,gamma);       %derivada de f(x)
error=100;          %para ingresar al while
i=0;                 %contador
ri=r0;
fprintf('iteraciones\t\t\t\tri\t\t\t\t\tri+1\t\t\t\terror\n');    	%el encabezado a imprimir
fprintf('%i\t\t\t\t%4.11f\t\t\t\t---------\t\t\t\t-----------\n',i,r0); %imprime nuestros datos inciales
while(error>=tol && i<=n)
   i=i+1;
   rim1=ri-(f(ri)/df(ri)); %hallamos la nueva raiz
   error=abs(((ri-rim1)/rim1)*100);     				%hallamos el error al tratar de encontrar la raiz
   fprintf('%i\t\t\t\t%4.11f\t\t\t\t%4.11f\t\t\t\t%4.11f\n',i,r0,rim1,error); %se imprimen los valores hallados por el metodo newton
   ri=rim1; 								%en caso no termine el bucle se vuelve a repetir con una nueva raiz
end
fprintf('La raiz es %4.11f y el error es %4.11f\n',rim1,error);     	%se imprimen los datos finales

%Posición del efector final para apuntar al misil objetivo
Px=7.5*cos(beta);
Py=7.5*sin(beta);
Pz=7.5*sin(gamma)+2.100017631;

%Posición en la matriz T del efecto final para apuntar
T(1,4)=Px;
T(2,4)=Py;
T(3,4)=Pz;

robot.inversekinematic_fn = 'inversekinematic_anti_misiles(robot, T)';
qinv=inversekinematic(robot,T)
drawrobot3d(robot,qinv)