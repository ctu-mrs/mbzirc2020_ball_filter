function plot_ellipse(a, b, x0, y0, theta, varargin)

t = -pi:0.01:pi;
t = [t, t(1)];
X = eval_ellipse(a, b, x0, y0, theta, t);
xs = X(:, 1);
ys = X(:, 2);

plot(xs, ys, varargin{:});

end
