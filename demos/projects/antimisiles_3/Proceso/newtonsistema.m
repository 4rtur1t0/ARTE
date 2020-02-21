function y = newtonsistema

xo=[0;0];
syms a t
fname=[1000-4000*sin(a)*t+0.5*9.8*t^2;2000-4000*cos(a)*t];
fprima=jacobian(fname); %Derivada
tolerancia=1; %Parámetro que determina cuando parar
maxiter=100; %Iteraciones
iter=1;
f=inline(fname); %Objeto tipo fila de fname
jf=inline(fprima); %Objeto tipo fila de fprima
error=norm(f(xo(1),xo(2)),2); %Cálculo del error
fprintf('error=%12.8f\n', error);

while error>=tolerancia
    fxo=f(xo(1),xo(2));
    fpxo=jf(xo(1),xo(2));
    x1=xo-inv(fpxo)*fxo;
    fx1=f(x1(1),x1(2));
    error=norm((fx1),2);
    fprintf('Iter %2d raiz x=(%14.9f,%14.9f) f(x)=(%14.9f,%14.9f)\n',iter,x1(1),x1(2),fx1(1),fx1(2));
  
if iter>maxiter
    fprintf('Numero maximo de iteraciones excedido \n');
 return;
end;
xo=x1;
iter=iter+1;
end;