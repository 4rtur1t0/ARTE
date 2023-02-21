% DIRECT JACOBIAN DEMO
function jacobian_planar_3gdl

%a)
q1=0
q2 = pi/2
q3 = pi/2
J = [-sin(q1)-sin(q1+q2)-sin(q1+q2+q3)  -sin(q1+q2)-sin(q1+q2+q3) -sin(q1+q2+q3);
     cos(q1)+cos(q1+q2)+cos(q1+q2+q3)    cos(q1+q2)+cos(q1+q2+q3)  cos(q1+q2+q3);
      0 0 0;
      0 0 0;
      0 0 0;
      1 1 1];
 f = [ 1 1 1 1 1 1]';
 
 tau = J'*f
 
 %b)
q1=pi/4
q2 = pi/4
q3 = pi/4
J = [-sin(q1)-sin(q1+q2)-sin(q1+q2+q3)  -sin(q1+q2)-sin(q1+q2+q3) -sin(q1+q2+q3);
     cos(q1)+cos(q1+q2)+cos(q1+q2+q3)    cos(q1+q2)+cos(q1+q2+q3)  cos(q1+q2+q3);
      0 0 0;
      0 0 0;
      0 0 0;
      1 1 1];
  
 tau = [1 1 1]';
  
 f = J*inv(J'*J)*tau
 
%c)
q1=pi/2
q2 = pi/2
q3 = pi/2
syms q1 q2 q3 real
J = [-sin(q1)-sin(q1+q2)-sin(q1+q2+q3)  -sin(q1+q2)-sin(q1+q2+q3) -sin(q1+q2+q3);
     cos(q1)+cos(q1+q2)+cos(q1+q2+q3)    cos(q1+q2)+cos(q1+q2+q3)  cos(q1+q2+q3)];
  
 %Jp = pinv(J);
 Jp = J'*inv(J*J');
 P = (eye(3) - Jp*J)
 P = simplify(P)
 qdn = P*[0 0 1]'
 qdn = simplify(qdn)

 
 

