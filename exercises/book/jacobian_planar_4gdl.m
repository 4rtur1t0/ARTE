% DIRECT JACOBIAN DEMO
function jacobian_planar_4gdl

%a)
q1=pi/4
q2 = pi/4
q3 = pi/4
q4 = pi/4
J = [-sin(q1)-sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4),  -sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2+q3+q4);
     cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),    cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),  cos(q1+q2+q3)+cos(q1+q2+q3+q4), cos(q1+q2+q3+q4);      
      1, 1, 1, 1]

  v = [1, 1, 1]
  
  Jp = J'*inv(J*J')
  
  qd1 = Jp*v'
  
  [u, s, v] = svd(J)
  
  qd2 = qd1 + v(:, 4)
  
  v1 = J*qd1
  
  v2 = J*qd2
  
  % ESFUERZOS
  J = [-sin(q1)-sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4),  -sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2+q3+q4);
     cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),    cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),  cos(q1+q2+q3)+cos(q1+q2+q3+q4), cos(q1+q2+q3+q4);      
      0, 0, 0, 0;
      0, 0, 0, 0;
      0, 0, 0, 0;
      1, 1, 1, 1]
  %esfuerzos a)
  f = [1 1 1 1 1 1]
  tau = J'*f'
  
  %esfuerzos b) 
  f = [-0.5 -0.5 0 0 0 -2]
  tau = J'*f'
  
  %esfuerzos c)  
  J = [-sin(q1)-sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4),  -sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2+q3+q4);
    cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),    cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),  cos(q1+q2+q3)+cos(q1+q2+q3+q4), cos(q1+q2+q3+q4);           
     1, 1, 1, 1]
  tau = [1 2 3 4]  
  fn = inv(J*J')*J*tau'
  