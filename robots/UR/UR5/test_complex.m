L2 = 0.425
L3 = 0.39225

u1 = (L2^2-L2*L3)/L2
u2 = (L2^2+L2*L3)/L2

R = 0.001:0.001:10;
cbeta = []
ceta = []
real = []
for i=1:length(R),
    a = isreal(acos((L2^2+R(i)^2-L3^2)/(2*L2*R(i))));
    b = isreal(acos((L2^2+L3^2-R(i)^2)/(2*L2*L3)));
    real = [real a&&b];
    
    %cbeta =[cbeta isreal(acos((L2^2+R(i)^2-L3^2)/(2*L2*R(i))))];
    %ceta = [ceta isreal(acos((L2^2+L3^2-R(i)^2)/(2*L2*L3)))];
end

figure
plot(R, real)