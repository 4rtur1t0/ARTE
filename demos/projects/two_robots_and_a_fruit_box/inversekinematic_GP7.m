
function q = inversekinematic_GP7(robot, T)


    q=zeros(6,8);


    d = eval(robot.DH.d);
    L6=abs(d(6));



    Px=T(1,4);
    Py=T(2,4);
    Pz=T(3,4);


    W = T(1:3,3);


    Pm = [Px Py Pz]' + L6*W; 


    q1=atan2(Pm(2), Pm(1));


 
    q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);


    q2_2=solve_for_theta2(robot, [q1+ pi 0 0 0 0 0 0], Pm);


    q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);

    %%%%solver for q3 for both cases
    q3_2=solve_for_theta3(robot, [q1+ pi 0 0 0 0 0 0], Pm);



    q = [q1         q1         q1        q1       q1+pi   q1+pi   q1+pi   q1+pi;   
        q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
        q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
        0          0          0         0         0      0       0       0;
        0          0          0         0         0      0       0       0;
        0          0          0         0         0      0       0       0];


    q=real(q);



    q(1,:) = normalize(q(1,:));
    q(2,:) = normalize(q(2,:));

    for i=1:2:size(q,2)
        qtemp = solve_spherical_wrist(robot, q(:,i), T, 1,'geometric'); 
        qtemp(4:6)=normalize(qtemp(4:6));
        q(:,i)=qtemp;
        
        qtemp = solve_spherical_wrist(robot, q(:,i), T, -1, 'geometric'); 
        qtemp(4:6)=normalize(qtemp(4:6));
        q(:,i+1)=qtemp;
    end
end


function q2 = solve_for_theta2(robot, q, Pm)


theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);


L2=abs(a(2));
L3=abs(d(4));
A2 = abs(a(3));


L4 = sqrt(A2^2 + L3^2);


T01=dh(robot, q, 1);


p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(p1(2), p1(1));
gamma = real(acos((L2^2+r^2-L4^2)/(2*r*L2)));


q2(1) =  beta + gamma - pi/2; 
q2(2) =  beta - gamma - pi/2; 

end




function q3 = solve_for_theta3(robot, q, Pm)


theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);


L2=abs(a(2));
L3=abs(d(4));

A2 = abs(a(3));


L4 = sqrt(A2^2 + L3^2);


phi=real(acos((A2^2+L4^2-L3^2)/(2*A2*L4)));


T01=dh(robot, q, 1);


p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

teta = real(acos((L2^2 + L4^2 - r^2)/(2*L2*L4)));


q3(1) = phi + teta - pi; 
q3(2) = phi - teta - pi;
end
