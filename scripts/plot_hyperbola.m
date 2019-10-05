function plot_hyperbola(a, b, x0, y0, theta, t, varargin)

Xs1 = a*cosh(t);
Xs2 = -a*cosh(t);
Ys = b*sinh(t);
xs1 = Xs1*cos(theta) + -Ys*sin(theta) + x0;
ys1 = Xs1*sin(theta) + Ys*cos(theta) + y0;
xs2 = Xs2*cos(theta) + -Ys*sin(theta) + x0;
ys2 = Xs2*sin(theta) + Ys*cos(theta) + y0;

plot(xs1, ys1, varargin{:}, xs2, ys2, varargin{:});

end
