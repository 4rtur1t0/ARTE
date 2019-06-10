function q = inversekinematic_irb1410(robot, T)

%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);

% %Evaluate the parameters
% theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
L6=d(6);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
x_f=T(1,4);
y_f=T(2,4);
z_f=T(3,4);
%
% %Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);
%
% % Pm: wrist position
Pm = [x_f y_f z_f]' - L6*W;


x=Pm(1);
y=Pm(2);
z=Pm(3);


%knowing that t=tan(q2+q3)/2

k=sqrt(pow(x,2)+pow(y,2));
sigma=1;
t_1_p=(sqrt(-1600000000*pow(z,4)+3040000000*pow(z,3)-80000*pow(z,2)*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-7737)+76000*z*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-25787)-1600000000*pow(k,4)*pow(sigma,4)+960000000*pow(k,3)*pow(sigma,3)+1918960000*pow(k,2)*pow(sigma,2)-618888000*k*sigma+562601831)-1200*(8*z+48*k*sigma-11))/(40000*pow(z,2)-95600*z+40000*pow(k,2)*pow(sigma,2)-2400*k*sigma+42757);
t_1_n=-(sqrt(-1600000000*pow(z,4)+3040000000*pow(z,3)-80000*pow(z,2)*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-7737)+76000*z*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-25787)-1600000000*pow(k,4)*pow(sigma,4)+960000000*pow(k,3)*pow(sigma,3)+1918960000*pow(k,2)*pow(sigma,2)-618888000*k*sigma+562601831)+1200*(8*z+48*k*sigma-11))/(40000*pow(z,2)-95600*z+40000*pow(k,2)*pow(sigma,2)-2400*k*sigma+42757);


sigma=-1;

t_2_p=(sqrt(-1600000000*pow(z,4)+3040000000*pow(z,3)-80000*pow(z,2)*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-7737)+76000*z*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-25787)-1600000000*pow(k,4)*pow(sigma,4)+960000000*pow(k,3)*pow(sigma,3)+1918960000*pow(k,2)*pow(sigma,2)-618888000*k*sigma+562601831)-1200*(8*z+48*k*sigma-11))/(40000*pow(z,2)-95600*z+40000*pow(k,2)*pow(sigma,2)-2400*k*sigma+42757);
t_2_n=-(sqrt(-1600000000*pow(z,4)+3040000000*pow(z,3)-80000*pow(z,2)*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-7737)+76000*z*(40000*pow(k,2)*pow(sigma,2)-12000*k*sigma-25787)-1600000000*pow(k,4)*pow(sigma,4)+960000000*pow(k,3)*pow(sigma,3)+1918960000*pow(k,2)*pow(sigma,2)-618888000*k*sigma+562601831)+1200*(8*z+48*k*sigma-11))/(40000*pow(z,2)-95600*z+40000*pow(k,2)*pow(sigma,2)-2400*k*sigma+42757);

t=[t_1_p, t_1_n, t_2_p,t_2_n];

for i=1:length(t)
    
    if i==1 || i==2
        sigma=1;
    else
        sigma=-1;
    end
    
    suma23=2*atan(t(i));
    
    sq2=(-0.12*sin(suma23)-0.72*cos(suma23)+0.475-z)/(0.6);%original
    cq2=(-0.12*cos(suma23)+0.72*sin(suma23)-0.15+sigma*k)/(0.6);%original
    

    
%     cq2=-(-0.12*sin(suma23)-0.72*cos(suma23)+0.475-z)/(0.6);
%     sq2=(-0.12*cos(suma23)+0.72*sin(suma23)-0.15+sigma*k)/(0.6);
    
    %q2=atan2(sq2,cq2);
    q2=2*atan(sq2/(cq2+1));
    
    q3=suma23-q2;
    
    
    sq1=y/(cos(q2)*(0.12*cos(q3)-0.72*sin(q3)+0.6)-sin(q2)*(0.72*cos(q3)+0.12*sin(q3))+0.15);
    cq1=x/(cos(q2)*(0.12*cos(q3)-0.72*sin(q3)+0.6)-sin(q2)*(0.72*cos(q3)+0.12*sin(q3))+0.15);
    
    %q1=atan2(sq1,cq1);
    q1=2*atan(sq1/(cq1+1));
    
    
    solution(i).q1=q1;
    solution(i).q2=q2;
    solution(i).q3=q3;
    
    solution(i+4).q1=q1;
    solution(i+4).q2=q2;
    solution(i+4).q3=q3;
    
    
    %% q1, q2, q3 already calculated , to compute q4, q5, q6
    
    a_03=[(cos(q1)*(cos(q2)*cos(q3)-sin(q2)*sin(q3))),(sin(q1)),(-cos(q1)*(cos(q2)*sin(q3)+sin(q2)*cos(q3))),cos(q1)*(cos(q2)*(3*cos(q3)/25+3/5)-3*sin(q2)*sin(q3)/25+3/20);
        (sin(q1)*(cos(q2)*cos(q3)-sin(q2)*sin(q3))),(-cos(q1)),(-sin(q1)*(cos(q2)*sin(q3)+sin(q2)*cos(q3))),(sin(q1)*(cos(q2)*(3*cos(q3)/25+3/5)-3*sin(q2)*sin(q3)/25+3/20));
        (-cos(q2)*sin(q3)-sin(q2)*cos(q3)),0,(sin(q2)*sin(q3)-cos(q2)*cos(q3)),(-3*cos(q2)*sin(q3)/25-sin(q2)*(3*cos(q3)/25+3/5)+(19/40));
        0,0,0,1];
    %
    % a_03_mat=[a_03(1), a_03(2),a_03(3),a_03(4);
    %     a_03(5), a_03(5),a_03(7),a_03(8);
    %     a_03(9), a_03(10),a_03(11),a_03(12);
    %     a_03(13), a_03(14),a_03(15),a_03(16)];
    
    a_03_inv=inv(a_03);
    
    
    Q=a_03_inv*T;
    
    %a_36=[[cos(q4)*cos(q5)*cos(q6)-sin(q4)*sin(q6),-cos(q4)*cos(q5)*sin(q6)-sin(q4)*cos(q6),-cos(q4)*sin(q5),-0.085*cos(q4)*sin(q5)],[cos(q4)*sin(q6)+sin(q4)*cos(q5)*cos(q6),cos(q4)*cos(q6)-sin(q4)*cos(q5)*sin(q6),-sin(q4)*sin(q5),-0.085*sin(q4)*sin(q5)],[sin(q5)*cos(q6),-sin(q5)*sin(q6),cos(q5),0.085*cos(q5)+0.72],[0,0,0,1]]
    
    %primera solucion
    
    q5=acos(Q(3,3));
    solution(i).q5=q5;
    
    if abs(sin(q5))<0.00001 %caso singular
        
        
        
        if cos(q5)>0 %cos = 1 ---->  q4+q6
            
            solution(i).q4=0;
            %solution(i).q6=atan2(Q(1,1),Q(2,1));
            solution(i).q6 =atan(Q(2,1)/(-(Q(1,1))+1));
             
            
        else   %cos = -1 ---->  q4-q6
            
            solution(i).q4=0;
            %solution(i).q6=atan2(Q(1,1),-Q(2,1));
           solution(i).q6=atan(Q(2,1)/((-Q(1,1))+1));
            
        end
        
    else %caso normal
        
        %solution(i).q6=atan2((Q(3,2)/-sin(q5)),((Q(3,1)/sin(q5))));
        solution(i).q6=2*atan((Q(3,2)/-sin(q5))/(((Q(3,1)/sin(q5)))+1));
        
        %solution(i).q4=atan2((Q(2,3)/-sin(q5)),((Q(1,3)/-sin(q5))));
        solution(i).q4=2*atan((Q(2,3)/-sin(q5))/(((Q(1,3)/-sin(q5)))+1));
    end
    
    %segunda solucion
    q5=-acos(Q(3,3));
    solution(i+4).q5=q5;
    
    
    if abs(sin(q5))<0.00001 %caso singular
        
        if cos(q5)>0 %cos = 1 ---->  q4+q6
            
            solution(i+4).q4=0;
            %solution(i).q6=atan2(Q(1,1),Q(2,1));
             solution(i+4).q6=atan(Q(2,1)/((-Q(1,1))+1));
            
        else   %cos = -1 ---->  q4-q6
            
            solution(i+4).q4=0;
            %solution(i).q6=atan2(Q(1,1),-Q(2,1));
           solution(i+4).q6=atan(Q(2,1)/((-Q(1,1))+1));
            
        end
    else
        
        %solution(i+4).q6=atan2((Q(3,2)/-sin(q5)),((Q(3,1)/sin(q5))));
        solution(i+4).q6=2*atan((Q(3,2)/-sin(q5))/(((Q(3,1)/sin(q5)))+1));
        
        %solution(i+4).q4=atan2((Q(2,3)/-sin(q5)),((Q(1,3)/-sin(q5))));
        solution(i+4).q4=2*atan(((Q(2,3)/-sin(q5)))/(((Q(1,3)/-sin(q5)))+1));
               
    end
    
%    Corrección por DH
 solution(i).q2=solution(i).q2+pi/2;
 solution(i+4).q2=solution(i+4).q2+pi/2;
 
 if solution(i).q6>0.00001
 solution(i).q6=solution(i).q6-pi;
 solution(i+4).q6=solution(i+4).q6-pi;
 end
 

 
end

for s=1:length(solution)
        
        sol=[solution(s).q1,solution(s).q2, solution(s).q3,solution(s).q4,solution(s).q5,solution(s).q6];
        if isreal(sol)
        q(:,s)=normalize(sol);
        else
            q(:,s)=sol;
        end
end
