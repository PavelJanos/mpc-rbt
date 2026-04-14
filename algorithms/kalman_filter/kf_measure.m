function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

if isempty(mu) || isempty(sigma) || isempty(z)
    new_mu = mu;
    new_sigma = sigma;
    return;
end

z = z(:);
if any(~isfinite(z))
    new_mu = mu;
    new_sigma = sigma;
    return;
end

C = kf.C;
Q = kf.Q;

if any(~isfinite(mu)) || any(~isfinite(sigma(:)))
    new_mu = mu;
    new_sigma = sigma;
    return;
end

% Innovation.
y = z - C * mu;
S = C * sigma * C' + Q;
S = 0.5 * (S + S');

if any(~isfinite(S(:)))
    new_mu = mu;
    new_sigma = sigma;
    return;
end

% Robust inverse for near-singular innovation covariance.
rc = rcond(S);
if ~isfinite(rc) || rc < 1e-12
    S = S + 1e-9 * eye(size(S));
end
rc = rcond(S);
if ~isfinite(rc) || rc < 1e-12
    Sinv = pinv(S);
else
    Sinv = inv(S);
end
K = sigma * C' * Sinv;

% Corrected mean.
new_mu = mu + K * y;
new_mu(3) = wrap_to_pi_local(new_mu(3));

% Joseph form for numerical stability.
I = eye(size(sigma));
new_sigma = (I - K * C) * sigma * (I - K * C)' + K * Q * K';
new_sigma = 0.5 * (new_sigma + new_sigma');

end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2 * pi) - pi;
end

