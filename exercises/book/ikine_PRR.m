% DIRECT JACOBIAN DEMO
function ikine_PRR

% definimos una matriz T
T = [0 1 0 1;
     0 0 1 0;
     1 0 0 1;
     0 0 0 1];
 
 q = ikinePRR(T);
 
 
 function q = ikinePRR(T)
 B1 = 1;
 L1 = 1;
 L2 = 1; 
 sphi = T(1,1);
 cphi = T(3,1);
 phi = atan2(sphi, cphi);
 px=T(1,4);
 pz=T(3,4);
  
 k1=px - B1 - L2*sin(phi);
 k2=pz - L2*cos(phi);
 
 a=1;
 b=-2*k2;
 c= k1^2 + k2^2 - L1^2;
 q1 = (b+sqrt(b^2-4*a*c))/(2*a);
 q1_ = (-b-sqrt(b^2-4*a*c))/(2*a);
 
 q2 =atan2(k1, k2-q1) 
 q2_ =atan2(k1, k2-q1_)
 
 q3 = phi-q2;
 q3_ = phi-q2_;
 
 q = [q1 q1_;
     q2 q2_;
     q3 q3_];

 
