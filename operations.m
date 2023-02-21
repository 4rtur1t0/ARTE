

syms pmx pmy pmz q1 q2 L3 L2 L1


a = (pmx*cos(q1) + pmy*sin(q1)-L3*sin(q2)-L2)^2 
b = (pmz -L1 + L3*cos(q2))^2

a = expand(a)
b = expand(b)
