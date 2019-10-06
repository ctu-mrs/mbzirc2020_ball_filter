clear;
% syms a b x0 y0 theta t x y;

% assume([a b x0 y0 theta t x y], 'real');
% xe = a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + x0;
% ye = a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + y0;
% xe = a*cos(t);
% ye = b*sin(t);
% ang = atan2(ye, xe);

% f = (xe - x)^2 + (ye - y)^2;
% dfdt = simplify(diff(f, t));
% t_sol = solve(dfdt == 0, t);

a = 5.0;
b = 2.0;
x0 = 8.0;
y0 = -2.0;
th = 2.1;

x = 8;
y = -2.00001;
pt = [x;y];
rot_th = [cos(th), -sin(th);
          sin(th), cos(th)];
pt_tfd = rot_th'*(pt - [x0; y0]);
x_tfd = pt_tfd(1);
y_tfd = pt_tfd(2);

t = atan2(y_tfd*a, x_tfd*b);

figure;
hold on;

plot_ellipse(a, b, x0, y0, th);
plot([x0, x], [y0, y], 'k');
plot(x, y, 'rx');
X = eval_ellipse(a, b, x0, y0, th, t);
plot(X(1), X(2), 'gx');
axis equal;
