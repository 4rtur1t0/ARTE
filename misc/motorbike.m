function motorbike()
close all
R = 0.3; %m
M = 190; % kg
c0 = 120; % Nm
cut = 10000; % max rpm
cut = cut*2*pi/60;
c1 = c0/cut; % Nm/rad
G = 215/16; %ratio reduccion
figure, 
wm= 0:0.1:cut;
plot(wm, c0-c1*wm)
title('Torque speed curve')

A=c0*G/(M*R);
B=c1*G^2/(M*R^2);
t=0:0.01:10;

x = (A/B)*(1-exp(-B*t));
figure,
plot(t, x)