fprintf('METODO NEWTON-RAPHSON\n'); 			%TITULO
syms f(x) %crea una variable simbolica 
f(x)=input('Ingrese la funcion: ');            		% se ingresara la funcion
r0=input('ingrese la primera aproximacion: ');  	%se ingresa el primer punto o raiz
tol=input('Ingrese la tolerancia: ');           	%valor del error minimo aceptable
n=input('Ingrese numero maximo de iteraciones: '); 	%hasta cuantas iteraciones 
df=diff(f,x);       %derivada de f(x)
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