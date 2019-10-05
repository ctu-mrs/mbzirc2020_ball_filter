clear;
%% define some of them ellipse parameters in different forms

% canonic ellipse parametrization
el_a = 5.0;
el_b = 2.0;
el_x0 = 5.0;
el_y0 = 7.0;
el_th = pi/3;
% el_x0 = 0;
% el_y0 = 0;
% el_th = 0;

% canonic conic section parametrization
% A = el_a^2*sin(el_th)^2 + el_b^2*cos(el_th)^2;
% B = 2*(el_b^2 - el_a^2)*sin(el_th)*cos(el_th);
% C = el_a^2*cos(el_th)^2 + el_b^2*sin(el_th)^2;
% D = -2*A*el_x0 - B*el_y0;
% E = -B*el_x0 - 2*C*el_y0;
% F = A*el_x0^2 + B*el_x0*el_y0 + C*el_y0^2 - el_a^2*el_b^2;
% theta = [A, B, C, D, E, F];
% theta = theta/norm(theta);
% A = theta(1);
% B = theta(2);
% C = theta(3);
% D = theta(4);
% E = theta(5);
% F = theta(6);

A =   0.0906494;
B =   -0.128263;
C =   0.0165987;
D = -0.00860307;
E =    0.408893;
F =   -0.898771;

Aq = [
  A, B/2, D/2;
  B/2, C, E/2;
  D/2, E/2, F
];
A33 = Aq(1:2, 1:2);
detAq = det(Aq);
detA33 = det(A33);
V = eig(A33);
lam1 = V(1);
lam2 = V(2);
K = -detAq/detA33;
a = sqrt(abs(K/lam1));
b = sqrt(abs(K/lam2));

disp((B^2 - 4*A*C));
% asq = 2*(A*E^2 + C*D^2 - B*D*E + (B^2 - 4*A*C)*F)*((A + C) + sqrt((A - C)^2 + B^2))/(B^2 - 4*A*C)^2;
% a = sqrt(abs(asq));
% bsq = 2*(A*E^2 + C*D^2 - B*D*E + (B^2 - 4*A*C)*F)*((A + C) - sqrt((A - C)^2 + B^2))/(B^2 - 4*A*C)^2;
% b = sqrt(abs(bsq));
x0 = (2*C*D - B*E)/(B^2 - 4*A*C);
y0 = (2*A*E - B*D)/(B^2 - 4*A*C);
% th = atan2(B, A - C)/2;
th = atan2(C - A - sqrt((A-C)^2 + B^2), B);
if A < C
  th = th + pi/2;
end
% if a > b
%   th = th - pi/2;
%   c = a;
%   a = b;
%   b = c;
% end

t = -pi:0.1:pi;
figure;
hold on;
% plot_ellipse(el_a, el_b, el_x0, el_y0, el_th, t, 'bx');
plot_hyperbola(el_a, el_b, el_x0, el_y0, el_th, t, 'bx');
% plot_ellipse(a, b, x0, y0, th, t, 'ro');
plot_hyperbola(a, b, x0, y0, th, t, 'ro');

%% generate some actual points
Xs = el_a*cosh(t);
Ys = el_b*sinh(t);

err1 = sum(abs(Xs.^2/(a^2) - Ys.^2/(b^2) - 1))

xs = Xs*cos(el_th) + -Ys*sin(el_th) + el_x0;
ys = Xs*sin(el_th) + Ys*cos(el_th) + el_y0;

%% check that the canonic conic section parametrization fits
xss = xs.^2;
xys = xs.*ys;
yss = ys.^2;

err2 = sum(abs(A*xss + B*xys + C*yss + D*xs + E*ys + F))

out = [xs', ys'];
csvwrite('conic_data.csv', out);
% gt = [A, B, C, D, E, F, a, b, x0, y0, th];
gt = [A, B, C, D, E, F, el_a, el_b, el_x0, el_y0, el_th];
csvwrite('conic.csv', gt);
