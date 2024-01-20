function [h] = plot_error_ellipse(P, mu, n, varargin)
%[h] = plot_error_ellipse(P, mu, n, varargin) Plot an error ellipse
%   Plot an error ellipse from a covariance matrix
%
% Arguments:
% P     Covariance matrix, 2x2
% mu    Mean of the distribution, 2x1
% n     Multiplier for sigma, scalar

% Determine ellipse
[U, S, ~] = svd(P)
ellipse_axes = sqrt(diag(S));

% plot ellipse
theta = linspace(0, 2*pi) ;
s1 = ellipse_axes(1) * n;
s2 = ellipse_axes(2) * n;
ellipse_pts = [s1 * cos(theta);
               s2 * sin(theta)];
for p = 1:size(ellipse_pts, 2)
    ellipse_pts(:, p) = U * ellipse_pts(:, p) + mu;
end
h = plot(ellipse_pts(1, :), ellipse_pts(2, :), varargin{:});

end