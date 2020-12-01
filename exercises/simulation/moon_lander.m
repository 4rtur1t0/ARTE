function moon_lander()
close all;

t0 = 0;
tfinal = 800;
initial_state = [0 160000 0 0 -100 0 15000]';

[t, x] = runge_kutta(@moonlander, initial_state, [t0 tfinal], 0.05);

figure, plot(x(1,:), x(2,:), '-b*')
title('Landing trajectory')

figure,
plot(t, x(3,:), 'k')
title('Attitude')

figure,
plot(t, x(5,:), 'r')
title('Landing speed')

figure,
plot(t, x(7,:), 'g')
title('Mass')




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a second order differential equation.
% Called from function exerciseB()
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = moonlander(t, x)
%target speed and x position
vtarget = -100;
xtarget = 0;



%constants
mass_min = 8000;
g = 1.6;
J = 100000;
ce=3.0*1000;
thlmax = 1000*0.5;
thtmax = 1000*44;
% compute control actions!
Kt = 300;
Kl = -0.000001;
vy = x(5);
posx = x(1);
posy = x(2);

if posy<3000
ut = Kt*(vtarget-vy);
ul = Kl*(xtarget-posx);
ul = sat(ul, -1, 1);
ut = sat(ut, 0, 1);
else
    ut=0;
    ul=0;
end

if x(7)<=mass_min
    Fl = 0;
    Ft = 0;
else
    Fl=thlmax*ul;
    Ft=thtmax*ut;
end

xd1 = x(4);
xd2 = x(5);
xd3 = x(6);
xd4 = (1/x(7))*(Fl*cos(x(3)) - Ft*sin(x(3)));
xd5 = (1/x(7))*(Fl*sin(x(3)) + Ft*cos(x(3))) - g;
xd6 = 4*Fl/J;

if x(7)<=mass_min
    % not changing mass
    display('ran out of fuel')
    % the derivative of mass is zero
    xd7 = 0 ;
else
    xd7 = -(abs(Fl/ce)+abs(Ft/ce));
end
xd = [xd1 xd2 xd3 xd4 xd5 xd6 xd7]';
return


function u= sat(u, min_val, max_val)
if u> max_val
    u = max_val;
end
if u < min_val
    u = min_val;
end

