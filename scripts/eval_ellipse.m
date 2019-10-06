function X = eval_ellipse(a, b, x0, y0, theta, t)

Xs = a*cos(t);
Ys = b*sin(t);
xs = Xs*cos(theta) + -Ys*sin(theta) + x0;
ys = Xs*sin(theta) + Ys*cos(theta) + y0;

X = [xs', ys'];

end
