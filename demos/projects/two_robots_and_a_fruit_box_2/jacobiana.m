function [qq]=jacobiana(robot,Xpunto)
    i=1;
    q = [-0.1440 -0.9810 -1.2830 0.0070 -0.0545 1.0050]';
    pause on;
    
    while i<99    
       J = compute_jacobian(robot,q); 
       qd = pinv(J)*Xpunto(:,i); 
       q = q + qd;
       
       qq(i,:)=q; %almacenamos en una matriz todas las 'q'
       
       i = i + 1;
    end
end