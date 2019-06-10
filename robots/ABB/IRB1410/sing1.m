%%Análisi de singularidades 

q=[0 0 0 0 0 0];

for i=1:10000
    %%q1
    q1= -170 + (170-(-170))*rand; %generar angulo aleatorio entre 0 y 360
    q1=deg2rad(q1);%pasamos a radianes
    %%q2
    q2= -70 + (70-(-70))*rand;
    q2=deg2rad(q2);
    %%q3
    q3= -65 + (70-(-65))*rand;
    q3=deg2rad(q3);
    %%q4
    q4= -150 + (150-(-150))*rand;
    q4=deg2rad(q4);
    %%q5
    q5= -115 + (115-(-115))*rand;
    q5=deg2rad(q5);
    %%q3
    q6= -300 + (300-(-300))*rand;
    q6=deg2rad(q6);
    
    q=[q1 q2 q3 q4 q5 q6];
    
    J = compute_jacobian(robot,q);
    if det(J)<0.00001
        disp(q)

    end
    [U,S,V] = svd(J);
    U=round(U,2)
    V=round(V,2)
    S=round(S,2)
end