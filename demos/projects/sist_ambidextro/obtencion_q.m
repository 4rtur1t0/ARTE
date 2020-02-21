function [q_general]=obtencion_q(robot,Xpunto)
    x=1;

	%Necesitamos poner esta el valor de q_inicial aqui tambien para evitar un error.
   %q = [-0.1440 -0.9810 -1.2830 0.0070 -0.0545 1.0050]';
    q = [-0.2827 -0.1396 -1.85 -1.1345 0.3037 0]';
    pause on;
    
    while x<99    
       
	J = compute_jacobian(robot,q); 
       qd = pinv(J)*Xpunto(:,x); 
       q = q + qd;

       q_general(x,:)=q;
       
       x = x + 1;
    end
end