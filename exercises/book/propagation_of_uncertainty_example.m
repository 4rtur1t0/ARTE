% Propagation of uncertainty as in Atlas
% Propagating uncertainty from node to node using relative motion.
%
%
function propagation_of_uncertainty_example
close all

poses = [0 0 0;
         10 0 0;
         10 10 pi/2;
         10 20 pi/2;
         0 20 pi;
         0 10 -pi/2;
         -10 -10 0];
n = 7
% figure, hold on
% plot(poses(:, 1), poses(:, 2), 'o')

S00 = diag([0.0 0.0 0.0])
sigmas = {}
sigmas{1} = S00
S = S00
for i=1:n-1
    S = propagate_uncertainty(poses(i, :), poses(i+1, :) , S)
    sigmas{i+1} = S;
end

% axis equal, hold on
% for i=1:n
%     plot_error_ellipse(poses(i,1:2), sigmas{i}(1:2, 1:2), 0.99)
% end

M = 500;
samples = zeros(M, 3);
sets = {};
sets{1} = samples;
for i=1:n-1
    samples = propagate_uncertainty_sampled(poses(i, :), poses(i+1, :) , samples);
    sets{i+1} = samples;
end

% plot everything
figure, hold on
plot(poses(:, 1), poses(:, 2), 'o')
axis equal, hold on
for i=1:n
   plot_error_ellipse(poses(i,1:2), sigmas{i}(1:2, 1:2), 0.99)
   plot(sets{i}(:,1), sets{i}(:,2), '.')
end



function Sres=propagate_uncertainty(posea, poseb, Si)

Sij = diag([0.1, 0.01, 0.001]);
Ti = T(posea(1), posea(2), posea(3));
Tj = T(poseb(1), poseb(2), poseb(3));
Tij = inv(Ti)*Tj;
xij = Tij(1,4);
yij = Tij(2,4);
thij = atan2(Tij(2,1), Tij(1,1));

[J1, J2] = Jacobians(posea(1), posea(2), posea(3), xij, yij, thij);

Sres = J1*Si*J1' + J2*Sij*J2';


function Set=propagate_uncertainty_sampled(ti, tj, Set)

Sjk = diag([0.1, 0.01, 0.001]);
Sjk = sqrt(Sjk);
Ti = T(ti(1), ti(2), ti(3));
Tj = T(tj(1), tj(2), tj(3));
Tij = inv(Ti)*Tj;
tij = t2v(Tij);
xij = tij(1);
yij = tij(2);
thij = tij(3);

for i=1:length(Set)
    xis = Set(i,1);
    yis = Set(i,2);
    this = Set(i, 3);
    Tis = T(xis, yis, this);
    % sample-based error propagation, add noise
    tx = xij + normrnd(0, Sjk(1,1));
    ty = yij + normrnd(0, Sjk(2,2));
    th = thij + normrnd(0, Sjk(3,3));
    Tijs = T(tx, ty, th);
    Tjs = Tis*Tijs;
    Set(i, :)= t2v(Tjs);
end




function A = T(x, y, th)

cth = cos(th);
sth = sin(th);

A=[cth  -sth  0   x;
   sth  cth   0   y;
    0   0     1   0;
    0   0     0   1];

function t = t2v(T)
x = T(1,4);
y = T(2,4);
th = atan2(T(2,1), T(1,1));
t = [x, y, th];


function [J1, J2] = Jacobians(xi, yi, thi, xij, yij, thij)

ci = cos(thi);
si = sin(thi);

J1=[1  0 -si*xij-ci*yij;
   0  1  ci*xij-si*yij;    
   0   0   1];

J2=[ci -si   0;
   si  ci   0;    
   0    0   1];


function plot_error_ellipse(mu, Sigma, p)
s = -2 * log(1 - p);
[V, D] = eig(Sigma * s);
t = linspace(0, 2 * pi, 50);
a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
plot(a(1, :) + mu(1), a(2, :) + mu(2));
