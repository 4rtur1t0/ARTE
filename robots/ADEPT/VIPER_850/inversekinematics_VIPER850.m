%%
%%
%% Authorship: Arturo Lopez Lopez. Universidad Miguel Hernández de Elche
%%
%% 
function Q=inversekinematics_VIPER850(robot,T)
%Obtención coordenadas x,y,z de la muñeca
d = eval(robot.DH.d);
l6=d(6);
P=T(1:3,4);
z6=T(1:3,3);
X=P-l6*z6;
x=X(1);
y=X(2);
z=X(3);
%Obtención q2 y q3
q2=[];
q3=[];
q1=[];
for sig =[1 -1]
    %Obtencion q2
    %Resolucion del polinomio de orden 2 en funcion de t obtenido con
    %Derive
    a=4*(375*sig*sqrt(x^2+y^2)-2500*x^2-2500*y^2-2*(1250*z^2+75*z-207));
    b=-365*(40*sig*sqrt(x^2+y^2)-3);
    c=5*(300*sig*sqrt(x^2+y^2)-2000*x^2-2000*y^2-2000*z^2+2800*z-647);
    t1=(-b+sqrt(b^2-4*a*c))/(2*a);
    t2=(-b-sqrt(b^2-4*a*c))/(2*a);
    q2_1=2*atan(t1);
    q2_2=2*atan(t2);
    %Obtencion q3 (Una vez se tiene q2)
    % alpha=q2+q3+atan(2/9)
    cos_alpha1=0.8797735233*sin(q2_1)+0.06025846050*(40*sig*sqrt(x^2+y^2)-3);
    sin_alpha1=0.01205169210*(200*z-67)-0.8797735233*cos(q2_1);
    cos_alpha2=0.8797735233*sin(q2_2)+0.06025846050*(40*sig*sqrt(x^2+y^2)-3);
    sin_alpha2=0.01205169210*(200*z-67)-0.8797735233*cos(q2_2);
    %Comprobar
    if abs(cos_alpha1+1) < 0.000001 && abs(cos_alpha2+1) < 0.000001
        q3_1=pi-q2_1-atan(2/9);
        q3_2=pi-q2_2-atan(2/9); 
    elseif abs(cos_alpha2+1) < 0.000001
        q3_1=2*atan(sin_alpha1/(1+cos_alpha1))-q2_1-atan(2/9);
        q3_2=pi-q2_2-atan(2/9);
    elseif abs(cos_alpha1+1) < 0.000001
        q3_1=pi-q2_1-atan(2/9);
        q3_2=2*atan(sin_alpha2/(1+cos_alpha2))-q2_2-atan(2/9);
    else
        q3_1=2*atan(sin_alpha1/(1+cos_alpha1))-q2_1-atan(2/9);
        q3_2=2*atan(sin_alpha2/(1+cos_alpha2))-q2_2-atan(2/9);
    end
    q2=[q2 q2_1 q2_2];
    q3=[q3 q3_1 q3_2];
end
%Obtencion q1
for i=1:length(q2)
    a1=cos(q2(i))*(0.405*cos(q3(i))-0.09*sin(q3(i)))-sin(q2(i))*(0.09*cos(q3(i))+0.405*sin(q3(i))+0.365)+0.075;
    cos_q1=x/a1;
    sin_q1=y/a1;
    if abs(cos_q1+1) < 0.000001
        q1(i) = pi;
    else
       q1(i)=2*atan(sin_q1/(1+cos_q1));
    end
end
%Obtencion q4, q5 y q6
Qm=[q1;q2;q3];
Q=[Qm Qm;zeros(3,8)];
for i=1:length(Qm(1,:))
    R03=[- cos(Qm(1,i))*(cos(Qm(2,i))*sin(Qm(3,i)) + sin(Qm(2,i))*cos(Qm(3,i))), sin(Qm(1,i)), cos(Qm(1,i))*(cos(Qm(2,i))*cos(Qm(3,i)) - sin(Qm(2,i))*sin(Qm(3,i))); - sin(Qm(1,i))*(cos(Qm(2,i))*sin(Qm(3,i)) + sin(Qm(2,i))*cos(Qm(3,i))), - cos(Qm(1,i)), sin(Qm(1,i))*(cos(Qm(2,i))*cos(Qm(3,i)) - sin(Qm(2,i))*sin(Qm(3,i))); cos(Qm(2,i))*cos(Qm(3,i)) - sin(Qm(2,i))*sin(Qm(3,i)), 0, cos(Qm(2,i))*sin(Qm(3,i)) + sin(Qm(2,i))*cos(Qm(3,i))];
    R06=T(1:3,1:3);
    RR=R03'*R06;
    if abs(RR(3,3)^2-1) < 0.000001
        q5_1=acos(RR(3,3));
        q5_2=q5_1;
    else
        q5_1=acos(RR(3,3));
        q5_2=2*pi-q5_1;

    end
    if abs(cos(q5_1)-1) < 0.000001
        %Al ser q5 igual a 0, se obtiene la suma de q4 y q6
        if abs(RR(1,1)+1) < 0.000001
            q4masq6=pi;
        else
            q4masq6=2*atan(RR(2,1)/(1+RR(1,1)));
        end
        %Se fija q4 igual a 0
        q4_1=0.0000;
        q4_2=0.0000;
        q6_1=q4masq6-q4_1;
        q6_2=q6_1;
    elseif abs(cos(q5_1)+1) < 0.000001
        %Al ser q5 igual a pi, se obtiene la resta de q4 y q6
        if abs((-RR(1,1))+1) < 0.000001
            q4menosq6=pi;
        else
            q4menosq6=2*atan((-RR(2,1))/(1+(-RR(1,1))));
        end
        %Se fija q4 igual a 0
        q4_1=0.0000;
        q4_2=0.0000;
        q6_1=q4menosq6+q4_1;
        q6_2=q6_1;
    else
        if abs((RR(1,3)/sin(q5_1))+1) < 0.000001 && abs((RR(1,3)/sin(q5_2))+1) < 0.000001
            q4_1=pi;
            q4_2=pi;
        elseif abs((RR(1,3)/sin(q5_1))+1) < 0.000001
            q4_1=pi;
            q4_2=2*atan((RR(2,3)/sin(q5_2))/(1+(RR(1,3)/sin(q5_2))));
        elseif abs((RR(1,3)/sin(q5_2))+1) < 0.000001
            q4_1=2*atan((RR(2,3)/sin(q5_1))/(1+(RR(1,3)/sin(q5_1))));
            q4_2=pi;
        else
            q4_1=2*atan((RR(2,3)/sin(q5_1))/(1+(RR(1,3)/sin(q5_1))));
            q4_2=2*atan((RR(2,3)/sin(q5_2))/(1+(RR(1,3)/sin(q5_2)))); 
        end
        if abs((RR(3,1)/-sin(q5_1))+1) < 0.000001 && abs((RR(3,1)/-sin(q5_2))+1) < 0.000001
            q6_1=pi;
            q6_2=pi;
        elseif abs((RR(3,1)/-sin(q5_1))+1) < 0.000001
            q6_1=pi;
            q6_2=2*atan((RR(3,2)/sin(q5_2))/(1+(RR(3,1)/-sin(q5_2)))); 
        elseif abs((RR(3,1)/-sin(q5_2))+1) < 0.000001
            q6_1=2*atan((RR(3,2)/sin(q5_1))/(1+(RR(3,1)/-sin(q5_1))));
            q6_2=pi;
        else
            q6_1=2*atan((RR(3,2)/sin(q5_1))/(1+(RR(3,1)/-sin(q5_1))));
            q6_2=2*atan((RR(3,2)/sin(q5_2))/(1+(RR(3,1)/-sin(q5_2)))); 
        end
    end
    Q(4,i)=q4_1;
    Q(4,i+4)=q4_2;
    Q(5,i)=q5_1;
    Q(5,i+4)=q5_2;
    Q(6,i)=q6_1;
    Q(6,i+4)=q6_2;
end
