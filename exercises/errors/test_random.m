global z1
global generate
close all

%init randomizer
z1 = rand();
generate = 0;
mu=0;
sigma = 2;

M=1500; %number of samples

puntos=[]
for i=1:M,
    gauss = generate_gaussian(mu, sigma);
    puntos=[puntos; gauss];
end
figure, 


plot(puntos, 'r.')
figure
hist(puntos,50)