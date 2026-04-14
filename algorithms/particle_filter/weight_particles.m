function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);
weights = zeros(N, 1);

if isempty(lidar_distances) || all(~isfinite(lidar_distances))
    weights = ones(N, 1) / N;
    return;
end

% Use a softer Gaussian likelihood on per-channel residuals.
sigma_lidar = 0.22;
inv_two_sigma2 = 1 / (2 * sigma_lidar^2);
eps_w = 1e-12;
missing_penalty = 0.18;

z = lidar_distances(:)';
finite_z = isfinite(z);
if ~any(finite_z)
    weights = ones(N, 1) / N;
    return;
end

for i = 1:N
    h = particle_measurements(i, :);
    valid = finite_z & isfinite(h);

    if ~any(valid)
        weights(i) = eps_w;
        continue;
    end

    err = h(valid) - z(valid);
    cost = mean(err.^2);

    mismatch = xor(finite_z, isfinite(h));
    if any(mismatch)
        cost = cost + missing_penalty * mean(mismatch);
    end

    weights(i) = exp(-cost * inv_two_sigma2) + eps_w;
end

sw = sum(weights);
if sw <= 0 || ~isfinite(sw)
    weights = ones(N, 1) / N;
else
    weights = weights / sw;
end

end

