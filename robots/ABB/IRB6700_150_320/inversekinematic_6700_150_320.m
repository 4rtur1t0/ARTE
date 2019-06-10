function q = inversekinematic_6700_150_320(robot, T)
%Por comodidad, pasamos los datos de T a valores:
r11=T(1,1);
r12=T(1,2);
r13=T(1,3);
r14=T(1,4);
r21=T(2,1);
r22=T(2,2);
r23=T(2,3);
r24=T(2,4);
r31=T(3,1);
r32=T(3,2);
r33=T(3,3);
r34=T(3,4);
r41=T(4,1);
r42=T(4,2);
r43=T(4,3);
r44=T(4,4);

%Obtenemos la posición de la muñeca y pasamos los valores a (x, y , z) para
%mayor facilidad de substitución en las fórmulas. 
m=T(1:3,4)-0.2*T(1:3,3);
x=m(1);
y=m(2);
z=m(3);


%Definimos q2+q3 como q23
%Para abs(sin(q5))=0, definimos el valor de la suma de los ángulos q4 + q6=
%q46 o la resta si q5==pi; q4 - q6= q46. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Resolvemos para sigma = -1 %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s=-1;
%%%%%%%%%%%%%%%Resolvemos para el PRIMER valor de t obtenido en la substitución de Wierstrass%%%%%%%%%%%%%

t=(sqrt(4602050780496000*s*sqrt(x^2 + y^2)*(3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 10571067046206377) - 9313225743103027344000000000000*x^4 - 6103515624000000*x^2*(3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*y^4 - 6103515624000000*y^2*(3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*z^4 + 29057264318481445313280000000000*z^3 + 41856006710456624603575848000000*z^2 - 100652249691040557038777426880000*z + 45482537206388083676452724533871) - 1525878906000*(800*s*sqrt(x^2 + y^2) - 13*(490*z - 359)))/(7418823240972000*s*sqrt(x^2 + y^2) + 3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 3540039061920000*z + 535401152657533);
q23=2*atan(t);% Como aparece un 2* sólo obtenemos una solución. Si no estuviera tendríamos 2 soluciones.

%Rsolvemos sq2 y cq2 de las ecuaciones anteriores a la última eliminación.
sq2=(1.5925*cos(q23) - 0.2*sin(q23) + 0.377 - s*sqrt(x^2 + y^2))/1.28;
cq2=(z - 0.2*cos(q23) - 1.5925*sin(q23) - 0.78)/1.28;
%obtenemos q2 con la fórmula de atan2
if abs(sq2)>0.00001 || cq2>0
    q2=2*atan(sq2/(1+cq2)); % la fórmula de atan2 es válida
else
    q2 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end
%obtenemos q3
q3=q23-q2;
%obtenemos sq1 y sq2 de la primera eliminación
sq1=y/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));
cq1=x/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));
%obtenemos q1
if abs(sq1)>0.00001 || cq1>0
    q1=2*atan(sq1/(1+cq1)); % la fórmula es válida
else
    q1 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end

%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 1%%%%% cogiendo + acos.....
q5=acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        if abs(sq46)>0.00001 || cq46>0
            q46=2*atan(sq46/(1+cq46)); % la fórmula es válida
        else
            q46 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end
        
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
       
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        
    end                                 %%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,1)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 1 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%
                                        
                                        
                                        
%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 2%%%%% cogiendo - acos.....
q5=-acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        if abs(sq46)>0.00001 || cq46>0
            q46=2*atan(sq46/(1+cq46)); % la fórmula es válida
        else
            q46 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
    end
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,2)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 2 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%
                                        

%%%%%%%%%%%%%%%Resolvemos para el SEGUNDO valor de t obtenido en la substitución de Wierstrass%%%%%%%%%%%%%

t=-(sqrt(4602050780496000*s*sqrt(x^2 + y^2)*(3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 10571067046206377) - 9313225743103027344000000000000*x^4 - 6103515624000000*x^2*(3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*y^4 - 6103515624000000*y^2*(3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*z^4 + 29057264318481445313280000000000*z^3 + 41856006710456624603575848000000*z^2 - 100652249691040557038777426880000*z + 45482537206388083676452724533871) + 1525878906000*(800*s*sqrt(x^2 + y^2) - 13*(490*z - 359)))/(7418823240972000*s*sqrt(x^2 + y^2) + 3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 3540039061920000*z + 535401152657533);
q23=2*atan(t);
sq2=(1.5925*cos(q23) - 0.2*sin(q23) + 0.377 - s*sqrt(x^2 + y^2))/1.28;
cq2=(z - 0.2*cos(q23) - 1.5925*sin(q23) - 0.78)/1.28;

if abs(sq2)>0.00001 || cq2>0
    q2=2*atan(sq2/(1+cq2)); % la fórmula es válida
else
    q2 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end

q3=q23-q2;
sq1=y/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));
cq1=x/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));

if abs(sq1)>0.00001 || cq1>0
    q1=2*atan(sq1/(1+cq1)); % la fórmula es válida
else
    q1 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end

%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 3%%%%% cogiendo + acos.....
q5=acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        if abs(sq46)>0.00001 || cq46>0
            q46=2*atan(sq46/(1+cq46)); % la fórmula es válida
        else
            q46 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
    end
                                        %%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,3)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 3 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%
                                        
%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 4%%%%% cogiendo - acos.....
q5=-acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        q46=2*atan(sq46/(1+cq46));
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end        
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
    end                                 %%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,4)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 4 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Resolvemos para sigma = 1 %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s=1;
%%%%%%%%%%%%%%%Resolvemos para el PRIMER valor de t obtenido en la substitución de Wierstrass%%%%%%%%%%%%%
m=T(1:3,4)-0.2*T(1:3,3);
x=m(1);
y=m(2);
z=m(3);
t=(sqrt(4602050780496000*s*sqrt(x^2 + y^2)*(3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 10571067046206377) - 9313225743103027344000000000000*x^4 - 6103515624000000*x^2*(3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*y^4 - 6103515624000000*y^2*(3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*z^4 + 29057264318481445313280000000000*z^3 + 41856006710456624603575848000000*z^2 - 100652249691040557038777426880000*z + 45482537206388083676452724533871) - 1525878906000*(800*s*sqrt(x^2 + y^2) - 13*(490*z - 359)))/(7418823240972000*s*sqrt(x^2 + y^2) + 3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 3540039061920000*z + 535401152657533);
q23=2*atan(t);
sq2=(1.5925*cos(q23) - 0.2*sin(q23) + 0.377 - s*sqrt(x^2 + y^2))/1.28;
cq2=(z - 0.2*cos(q23) - 1.5925*sin(q23) - 0.78)/1.28;

if abs(sq2)>0.00001 || cq2>0
    q2=2*atan(sq2/(1+cq2)); % la fórmula es válida
else
    q2 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end

q3=q23-q2;
sq1=y/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));
cq1=x/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));

if abs(sq1)>0.00001 || cq1>0
    q1=2*atan(sq1/(1+cq1)); % la fórmula es válida
else
    q1 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end

%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 5%%%%% cogiendo + acos.....
q5=acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        if abs(sq46)>0.00001 || cq46>0
            q46=2*atan(sq46/(1+cq46)); % la fórmula es válida
        else
            q46 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
    end                                 %%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,5)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 5 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 6%%%%% cogiendo - acos.....
q5=-acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        if abs(sq46)>0.00001 || cq46>0
            q46=2*atan(sq46/(1+cq46)); % la fórmula es válida
        else
            q46 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
    end                                 %%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,6)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 6 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%Resolvemos para el SEGUNDO valor de t obtenido en la substitución de Wierstrass%%%%%%%%%%%%%

t=-(sqrt(4602050780496000*s*sqrt(x^2 + y^2)*(3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 10571067046206377) - 9313225743103027344000000000000*x^4 - 6103515624000000*x^2*(3051757812000000*y^2 + 3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*y^4 - 6103515624000000*y^2*(3051757812000000*z^2 - 4760742186720000*z - 9703580474082881) - 9313225743103027344000000000000*z^4 + 29057264318481445313280000000000*z^3 + 41856006710456624603575848000000*z^2 - 100652249691040557038777426880000*z + 45482537206388083676452724533871) + 1525878906000*(800*s*sqrt(x^2 + y^2) - 13*(490*z - 359)))/(7418823240972000*s*sqrt(x^2 + y^2) + 3051757812000000*x^2 + 3051757812000000*y^2 + 3051757812000000*z^2 - 3540039061920000*z + 535401152657533);
q23=2*atan(t);
sq2=(1.5925*cos(q23) - 0.2*sin(q23) + 0.377 - s*sqrt(x^2 + y^2))/1.28;
cq2=(z - 0.2*cos(q23) - 1.5925*sin(q23) - 0.78)/1.28;

if abs(sq2)>0.00001 || cq2>0
    q2=2*atan(sq2/(1+cq2)); % la fórmula es válida
else
    q2 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end

q3=q23-q2;
sq1=y/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));
cq1=x/(0.0005*(cos(q3)*(3185*cos(q2) - 400*sin(q2)) - sin(q3)*(400*cos(q2) + 3185*sin(q2)) - 2560*sin(q2) + 754));

if abs(sq1)>0.00001 || cq1>0
    q1=2*atan(sq1/(1+cq1)); % la fórmula es válida
else
    q1 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
end

%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 7%%%%% cogiendo + acos.....
q5=acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        if abs(sq46)>0.00001 || cq46>0
            q46=2*atan(sq46/(1+cq46)); % la fórmula es válida
        else
            q46 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
    end                                 %%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,7)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 7 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%Para hallar los 3 ángulos de la muñeca --> %%%%SOLUCIÓN 8%%%%% cogiendo - acos.....
q5=-acos(r13*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r23*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r43*cos(q3) + 20*(39*r43 - 50*r33)*sin(q3)) + cos(q2)*(sin(q2)*(754*r43*cos(q3)^2 + 40*(39*r43 - 50*r33)*sin(q3)*cos(q3) - 377*r43) + 1280*r43*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - 1280*r43*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)));
    if abs(sin(q5))<0.00001
        cq46=-(r11*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r21*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r41*cos(q3)^3 + 20*(117*r41 - 200*r31)*sin(q3)*cos(q3)^2 - 754*r41*cos(q3) + 20*(50*r31 - 39*r41)*sin(q3)) + 40*r41*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r31*cos(q3)^2 + 1131*r41*sin(q3)*cos(q3) + 20*(117*r41*sin(q3)^2 - 50*r31 - 39*r41)) - 80*r41*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)^2) + sin(q2)*cos(q3)*(13*r41*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r41*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r31*sin(q3)*cos(q3) + 377*r41))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2);
        sq46 = r12*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r22*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r42*cos(q3)^3 + 20*(117*r42 - 200*r32)*sin(q3)*cos(q3)^2 - 754*r42*cos(q3) + 20*(50*r32 - 39*r42)*sin(q3)) + 40*r42*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r32*cos(q3)^2 + 1131*r42*sin(q3)*cos(q3) + 20*(117*r42*sin(q3)^2 - 50*r32 - 39*r42)) - 80*r42*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)^2) + sin(q2)*cos(q3)*(13*r42*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r42*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r32*sin(q3)*cos(q3) + 377*r42))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2;
        if abs(sq46)>0.00001 || cq46>0
            q46=2*atan(sq46/(1+cq46)); % la fórmula es válida
        else
            q46 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        q4=0;
        if cos(q5)> 0 %Entonces q46 es la suma de los ángulos q4 y q6
            q6=q46-q4;
        else          %Si q5 está cercano a pi, entonces q46 es la resta de los ángulos q4 y q6 (q46=q4-q6)
            q6=q4-q46;
        end
    else
        sq6= (r12*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r22*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r42*cos(q3) + 20*(39*r42 - 50*r32)*sin(q3)) + cos(q2)*(sin(q2)*(754*r42*cos(q3)^2 + 40*(39*r42 - 50*r32)*sin(q3)*cos(q3) - 377*r42) + 1280*r42*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r32 - 39*r42)*cos(q3) + 377*r42*sin(q3)) - 1280*r42*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/(-sin(q5));
        cq6= (r11*cos(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) + r21*sin(q1)*(2*cos(q2)^2*sin(q3)*cos(q3) + sin(q2)*cos(q2)*(2*cos(q3)^2 - 1) - sin(q3)*cos(q3))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)) - 0.001*(cos(q2)^2*sin(q3)*(377*r41*cos(q3) + 20*(39*r41 - 50*r31)*sin(q3)) + cos(q2)*(sin(q2)*(754*r41*cos(q3)^2 + 40*(39*r41 - 50*r31)*sin(q3)*cos(q3) - 377*r41) + 1280*r41*sin(q3)^2) - sin(q2)*cos(q3)*(sin(q2)*(20*(50*r31 - 39*r41)*cos(q3) + 377*r41*sin(q3)) - 1280*r41*sin(q3)))/(cos(q2)*sin(q3) + sin(q2)*cos(q3)))/sin(q5);
        if abs(sq6)>0.00001 || cq6>0
            q6=2*atan(sq6/(1+cq6)); % la fórmula es válida
        else
            q6 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
        cq4=(r13*cos(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + r23*sin(q1)*(cos(q2)^3*sin(q3)*(2*cos(q3)^2 - 1) + sin(q2)*cos(q2)^2*cos(q3)*(1 - 4*sin(q3)^2) - cos(q2)*sin(q3)*cos(q3)^2*(2*sin(q2)^2 + 1) - sin(q2)*cos(q3)^3)/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2 + 0.001*(cos(q2)^3*sin(q3)^2*(20*(100*r33 - 39*r43)*cos(q3) + 377*r43*sin(q3)) - cos(q2)^2*(sin(q2)*(1131*r43*cos(q3)^3 + 20*(117*r43 - 200*r33)*sin(q3)*cos(q3)^2 - 754*r43*cos(q3) + 20*(50*r33 - 39*r43)*sin(q3)) + 40*r43*sin(q3)^2*(32*cos(q3) + 5)) + cos(q2)*cos(q3)*(sin(q2)^2*(2000*r33*cos(q3)^2 + 1131*r43*sin(q3)*cos(q3) + 20*(117*r43*sin(q3)^2 - 50*r33 - 39*r43)) - 80*r43*sin(q2)*sin(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)^2) + sin(q2)*cos(q3)*(13*r43*sin(q2)^2*sin(q3)*(60*cos(q3) - 29*sin(q3)) - 40*r43*sin(q2)*cos(q3)*(32*cos(q3) + 5) - 1000*r33*sin(q3)*cos(q3) + 377*r43))/(cos(q2)*sin(q3) + sin(q2)*cos(q3))^2)/sin(q5);
        sq4=(r13*sin(q1) - r23*cos(q1))/sin(q5);
        if abs(sq4)>0.00001 || cq4>0
            q4=2*atan(sq4/(1+cq4)); % la fórmula es válida
        else
            q4 = pi; % hay que asignar directamente pi porque en este caso la formula Atan2 no es válida
        end
    end                                 %%%%%%%%%%%%%%%%%%%%%%%%%
q(1:6,8)=[q1,q2,q3,q4,q5,q6]; %%%%%%%%%% OBTENEMOS LA SOLUCIÓN 8 %%%%%%%%%%%%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%
% for i=1:8
%     q(:,i)=normalize(q(:,i));
% end
% SE NORMALIZAN LOS VALORES.
    for s=1:6
        for t=1:8
            if q(s,t)>pi
                q(s,t)=q(s,t)-2*pi;
            end
            if q(s,t)<-pi
                 q(s,t)=q(s,t)+2*pi;
            end
        end
    end

end