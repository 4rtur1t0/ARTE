close all

G=1/100;
figure, plot(tout, par_articulacion), ylabel('Par en articulacion (N·m)')
figure, plot(tout, par_motor), ylabel('Par en el motor (N·m)')
figure, plot(tout, velocidad),ylabel('Velocidad articular (rad/s)')
figure, plot(tout, 1/G*velocidad*60/(2*pi)),ylabel('Velocidad motor (r.p.m)')