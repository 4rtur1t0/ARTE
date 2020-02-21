function interception_libre

clear all;
objetivo=[5 2 0];
modo='par';
p0=[-8.970149e+03 8.259738e+03 0];
acimut=2.383670e+00;
elevacion=1.034366e+00;
td=3.236595e+01;


global g k m
%objetivo=[1 5 0];
Rmax_cohete = 15000;                       %Rangos de disparo del cohete
Rmin_cohete = 10000; 
H_max=500;
H_min=50;

v0_misil=500;                              %Velocidad del misil m/s  
assignin('base', 'v0_misil', v0_misil);
h=5000;                                    %Altura máxima alcanzada por el misil
H_max=500;
H_min=50;

rango=650;                                 %Rango efectivo de la bala
g = 9.81;                                  %m/s^ 2. gravedad
k = 0.0000005;                             %N*s/m friction coefficient
m=101/1000;                                %Masa de la bala
v0_bala=1035;                              %m/s initial speed
l_canon=2;                                 %Longitud del cañon M61 Vulcan Block 1B
h_canon=2;                                 %Altura de montaje, supuesta al no encontrar datos 

theta_min=degtorad(-25);                   %Angulos inicial y final de la elevación de la torreta, 
theta_max=degtorad(85);                    %datos recogidos de wikipedia,se utilizará la configuración Block 1B ya que es la que más rango da
vel_ang=3;                                 %V angular maxima en rad/s

precision=1;
t0 = 0;
sol=0;

assignin('base','precision',precision);
switch modo
    case 'par'
        mod= 1;
        assignin('base', 'mod', mod);
        assignin('base', 'u_lin', [1 0 0]);
        %%%%%%%%% Cálculo del punto aleatorio de disparo del cohete %%%%%%%%%%%%%%%%

        assignin('base', 'p0', p0);
        %%%%%%%%% Cálculo de los parámetros de la parábola %%%%%%%%%%%%%%%%


        d=norm(objetivo-p0);

        p2(1)= d/2;
        p2(2)= h;

        xl=[0:1:d];
        
        syms a b c ;
        [sol_a, sol_b, sol_c]=vpasolve( c==0, a*p2(1)^2 + b*p2(1) + c==p2(2), a*d^2 + b*d + c==0,[a, b, c]);
        yl=sol_a*xl.^2 + sol_b*xl + sol_c;
        assignin('base', 'sol_a', double(sol_a));
        assignin('base', 'sol_b', double(sol_b));
        assignin('base', 'sol_c', double(sol_c));
        
        fun_l=@(x) sqrt(1+(2*double(sol_a)*x+double(sol_b)).^2);
        l_total=integral(fun_l,0,d);
        tfinal=l_total/v0_misil;
        
        %%%%%%%%% Cálculo de la matriz de transformación homogénea para pasar de coordenadas locales de parábola a generales %%%%%%%%%%%%%%%%

        u_par=(objetivo-p0)/d;
        v=[1 0 0];

        alpha= atan2(norm(cross(u_par,v)),dot(u_par,v));
        if u_par(2)<0
            alpha=-alpha;
        end

        T=[cos(alpha) -sin(alpha) 0 p0(1); sin(alpha) cos(alpha) 0 p0(2); 0 0 1 0; 0 0 0 1];
        assignin('base', 'T', T);
        s=size(xl,2);
        v_c=[xl;zeros(1,s);yl;ones(1,s)];
        P=zeros(4,s);

        for i=1:s           %la s es el tamaño horizontal de la matriz en el tiempo
            P(:,i) =  T*v_c(:,i);

        end

        for i=1:1:s 
            if sqrt((P(1,i)^2)+(P(2,i)^2)+(P(3,i)^2))<=rango
                p_corte=[P(1,i) P(2,i) P(3,i)];
                index=i;
                break
            end    
        end
        
        l_corte=integral(fun_l,0,xl(index));
        td_aprox=l_corte/v0_misil;
        
    case 'lin'
        
        mod=0;
        assignin('base', 'mod', mod);
        
        sol_a=0;
        sol_b=0;
        sol_c=0;
        T=zeros(4,4);  
        assignin('base', 'T', T);
        assignin('base', 'sol_a', sol_a);
        assignin('base', 'sol_b', sol_b);
        assignin('base', 'sol_c', sol_c);
        
        %%%%%%%%% Cálculo del punto aleatorio de disparo del cohete %%%%%%%%%%%%%%%%
        assignin('base', 'p0', p0);
        %%%%%%%%% Cálculo de la recta %%%%%%%%%%%%%%%%%%%%%%%%%
        
        d=norm(objetivo-p0);
        u_lin=(objetivo-p0)/d;
        assignin('base', 'u_lin', u_lin);
        xl=[p0(1):(objetivo(1)-p0(1))/d:objetivo(1)];
        yl=[p0(2):(objetivo(2)-p0(2))/d:objetivo(2)];
        zl=[p0(3):(objetivo(3)-p0(3))/d:objetivo(3)];
        P=[xl;yl;zl];
        s=size(xl,2);
        tfinal=d/v0_misil;
      
        for i=1:1:s 
            if sqrt((P(1,i)^2)+(P(2,i)^2)+(P(3,i)^2))<=rango
                p_corte=[P(1,i) P(2,i) P(3,i)];
                break
            end    
        end
        
        td_aprox= norm(p_corte-p0)/v0_misil;
end

%%%%%%%%%%%%%%%%%%% Cálculo del acimut y elevación aproximada %%%%%%%%%%%%%%%%%%%%%%%%%%%%


ac_inicial=[1 0 0];                        %Suponemos que el cañon apunta inicialmente en la dirección positiva del eje x
ac_final=[p_corte(1) p_corte(2) 0];        %Proyección del punto en el que el misil entra en el radio de accion del cañon
delta_t=(tfinal-(td_aprox-0.6))/200;             %Intervalos de tiempo y angulo para simulación
ac_3d=vrrotvec(ac_inicial,ac_final);

assignin('base','ac_3d',ac_3d);
        
%%%%%%%%%%%%%%% Cálculo de la trayectoria de la bala %%%%%%%%%%%%%%%%%%%

                %RUNGE-KUTTA
        x0_bala=l_canon*cos(elevacion);
        y0_bala=l_canon*sin(elevacion)+h_canon;
        vx0 = v0_bala*cos(elevacion);
        vy0 = v0_bala*sin(elevacion);
        T_bala=[cos(acimut) -sin(acimut) 0 x0_bala; sin(acimut) cos(acimut) 0 y0_bala; 0 0 1 0; 0 0 0 1];
        assignin('base', 'T_bala', T_bala);
        
        [t, pv_bala] = runge_kutta(@parabolic_friction_square, [x0_bala y0_bala vx0 vy0]', [td tfinal], delta_t);
        pv_bala = pv_bala(:, 1:length(t));                   %pv_bala contiene posicion y velocidad de la vala en el eje x e y 
        
        p_bala=[pv_bala(1,:); pv_bala(2,:)];                 %Parámetros para el simulink
        ni=(td-t0)/delta_t;
        index_bala=[ones(1,int32(ni)) 1:size(p_bala,2)];
        
        assignin('base','index_bala',index_bala);
        assignin('base', 'px_bala', p_bala(1,:));
        assignin('base', 'py_bala', p_bala(2,:));
        
%%%%%%%%%%%%%%% Simulacion %%%%%%%%%%%%%%%%%%%

        simul=sim('torreta_anim','StopTime',num2str(tfinal), 'FixedStep', num2str(delta_t));
        dis_col=simul.get('dis_col');
        
        for i=1:length(dis_col)                                         %El siguiente bucle anidado evalua si ha habido un impacto cumpliendo restricciones tanto articulares
            if dis_col(i)<precision                                     %como de precisión
                tt=acimut/vel_ang + elevacion/vel_ang;
                if ((theta_min<elevacion && elevacion<theta_max) && (tt<td))
                    sol=sol+1;
                    fprintf('\n______________________________\n|\t\t\t\t\t\t\t|\n|\tSolucion nº: %d\t\t\t|\n|\t\t\t\t\t\t\t|\n______________________________\n',sol)                    
                    fprintf('\nImpacto con disparo de tipo %s con origen en [%d %d %d] y objetivo en [%d %d %d]' ,modo,p0(1),p0(2),p0(3),objetivo(1),objetivo(2),objetivo(3))
                    fprintf('\n Logrado con los siguientes parámetros de disparo:\n\t- Acimut: %d\n\t- Elevación: %d\n\t- Tiempo de disparo: %d\n\t- Distancia de colision: %d\n',acimut,elevacion,td,dis_col(i))

                    figure
                    hold on 
                    plot3(simul.get('x_misil'),simul.get('y_misil'),simul.get('z_misil'),'r')
                    plot3(simul.get('x_bala'),simul.get('y_bala'),simul.get('z_bala'),'b')
                    axis([-100 100 -100 100 0 200])
                    hold off
                    legend('Trayectoria del misil','Trayectoria de la bala')
                    
                else
                    fprintf('\nNo se cumplen las restricciones articulares necesarias para impacto\n')                   
                end                
            end            
        end

if sol==0
    fprintf('\nNo ha sido posible encontrar una solucion valida\n')
end

  

        
function xd = parabolic_friction_square(t, x)

global k g m
% We must return the solution of
% [dx1/dt; dx2/dt; dx3/dt dx4/dt]
vx = x(3);
vy = x(4);
%forces in both axes
sq = sqrt(vx^2+vy^2);
xd(1) = vx;
xd(2) = vy;
xd(3) = -(k/m)*vx*sq;
xd(4) = -(k/m)*vy*sq - g;
xd = xd(:);
function [tout, yout]=runge_kutta(f, y0, timespan, timestep)
%timestep for the integration
h=timestep;
t0 = timespan(1);
tfinal = timespan(2);

y = y0;
yout = y;                                         
tout=t0:h:tfinal;
for t=t0:h:tfinal-h,                             
    k1 = f(t, y);
    k2 = f(t + 0.5*h, y + 0.5*h*k1);
    k3 = f(t + 0.5*h, y + 0.5*h*k2);
    k4 = f(t + h, y + k3*h);

    y = y + h*(k1+2*k2+2*k3+k4)/6; 
    yout = [yout y];
end
