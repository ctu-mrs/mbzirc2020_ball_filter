clear;
%% define some of them ellipse parameters in different forms

% canonic ellipse parametrization
el_a = 3.0;
el_b = 2.0;
el_x0 = 5.0;
el_y0 = 7.0;
el_th = pi/3;
% el_x0 = 0;
% el_y0 = 0;
% el_th = 0;

% canonic conic section parametrization
A = el_a^2*sin(el_th)^2 + el_b^2*cos(el_th)^2;
B = 2*(el_b^2 - el_a^2)*sin(el_th)*cos(el_th);
C = el_a^2*cos(el_th)^2 + el_b^2*sin(el_th)^2;
D = -2*A*el_x0 - B*el_y0;
E = -B*el_x0 - 2*C*el_y0;
F = A*el_x0^2 + B*el_x0*el_y0 + C*el_y0^2 - el_a^2*el_b^2;
theta = [A, B, C, D, E, F];
theta = theta/norm(theta);
A = theta(1);
B = theta(2);
C = theta(3);
D = theta(4);
E = theta(5);
F = theta(6);

a = sqrt(2*(A*E^2 + C*D^2 - B*D*E + (B^2 - 4*A*C)*F)*((A + C) + sqrt((A - C)^2 + B^2)))/(B^2 - 4*A*C);
b = sqrt(2*(A*E^2 + C*D^2 - B*D*E + (B^2 - 4*A*C)*F)*((A + C) - sqrt((A - C)^2 + B^2)))/(B^2 - 4*A*C);
x0 = (2*C*D - B*E)/(B^2 - 4*A*C);
y0 = (2*A*E - B*D)/(B^2 - 4*A*C);
th = atan2(C-A-sqrt((A-C)^2+B^2), B);

t = 0:0.1:2*pi;
figure;
hold on;
plot_ellipse(el_a, el_b, el_x0, el_y0, el_th, t, 'bx');
plot_ellipse(a, b, x0, y0, th, t, 'ro');

%% generate some actual points
t = 0:0.1:2*pi;
Xs = el_a*cos(t);
Ys = el_b*sin(t);

err1 = sum(abs(Xs.^2/(a^2) + Ys.^2/(b^2) - 1))

xs = Xs*cos(el_th) + -Ys*sin(el_th) + el_x0;
ys = Xs*sin(el_th) + Ys*cos(el_th) + el_y0;

%% check that the canonic conic section parametrization fits
xss = xs.^2;
xys = xs.*ys;
yss = ys.^2;

err2 = sum(abs(A*xss + B*xys + C*yss + D*xs + E*ys + F))

out = [xs', ys'];
csvwrite('conic_data.csv', out);
csvwrite('conic.csv', [A, B, C, D, E, F]);
