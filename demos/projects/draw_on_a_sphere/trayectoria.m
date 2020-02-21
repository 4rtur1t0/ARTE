function [q,qd,v,w,tiempo]=trayectoria(T,robot)
global velocidad;
global delta_t;
for i=1:length(T);
    P(:,i)=T{i}(1:3,4);
    qq = inversekinematic_kuka_kr160_r1570_nanoC(robot,T{i});
    q_base(:,i)=qq(:,1);  
end
 t0=0;

q=[];
qd=[];
tiempo=[];
for i=1:length(q_base)-1;
    t1=abs(norm(P(i+1)-P(i))/(velocidad/100));
    t=[t0 t0+t1];
    q_t=[];
    qd_t=[];
    for h=1:6;
        [q_, qd_, time, k]=first_order([q_base(h,i) q_base(h,i+1)],[t(1) t(2)], delta_t);
        q_t=[q_t;q_];
        qd_t=[qd_t;qd_];
    end
    q=[q q_t];
    qd=[qd qd_t];
    tiempo=[tiempo time];
    t0=t0+t1;
end

for i=1:length(q)-1;

        
        %qqq{i-1} = inversekinematic_kuka_kr160_r1570_nanoC(robot,T{i});
        J = manipulator_jacobian(robot, q(:,i));
        vv=J*qd(:,i);
        v(:,i)=vv(1:3);
        w(:,i)=vv(4:6);
      %  Ti = directkinematic(robot, q);
    

end

end

function [q_t, qd_t, time, k]=first_order(q, t, delta_t)
%define the time matrix
A = [1 t(1); 
     1 t(2)];
b = [q(1) q(2)];
% compute the coefficients of k for a first order planner
k=inv(A)*b';

%define the time vector using delta_time
time=t(1):delta_t:t(2);
%the first order equation
q_t=k(1) + k(2)*time;
%return a constant speed
qd_t=k(2)*ones(1,length(time));
end