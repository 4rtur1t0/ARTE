% Propagation of uncertainty
function propagation_of_uncertainty_derivation
close all

syms xi yi thi xij yij thetaij

Ti = T(xi, yi, thi)
Tij = T(xij, yij, thetaij)
Tj = simplify(Ti*Tij)

J1 = Jacobianrel1(xi, yi, thi, xij, yij, thij)
J2 = Jacobianrel2(xi, yi, thi, xij, yij, thij)

Sij = diag([0.2, 0.1, 0.1])
S2 = J1*Si*J1' + J2*Sij*J2'


function A = T(x, y, th)

cth = cos(th);
sth = sin(th);

A=[cth  -sth  0   x;
   sth  cth   0   y;
    0   0     1   0;
    0   0     0   1];


function J = Jacobianrel1(xi, yi, thi, xij, yij, thij)

ci = cos(thi);
si = sin(thi);

J=[1  0 -si*xij-ci*yij;
   0  1  ci*xij-si*yij;    
   0   0   1];




function J = Jacobianrel2(xi, yi, thi, xj, yj, thj)

cth = cos(thij);
sth = sin(thij);

J=[cth -sth 0;
   sth  cth 0;    
   0   0   1];


function plotErrorEllipse(mu, Sigma, p)

    s = -2 * log(1 - p);

    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi, 50);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

    plot(a(1, :) + mu(1), a(2, :) + mu(2));
