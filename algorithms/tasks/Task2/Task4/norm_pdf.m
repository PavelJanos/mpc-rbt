function y = norm_pdf(x, mu, sigma)
%NORM_PDF Probability density function of a normal distribution.
% y = norm_pdf(x, mu, sigma)

if ~isscalar(mu) || ~isreal(mu)
    error('mu must be a real scalar.');
end
if ~isscalar(sigma) || ~isreal(sigma) || sigma <= 0
    error('sigma must be a positive real scalar.');
end

y = (1 / (sigma * sqrt(2 * pi))) * exp(-((x - mu).^2) / (2 * sigma^2));

end
