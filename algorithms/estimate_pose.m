function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

particles = public_vars.particles;
weights = [];
if isfield(public_vars, 'particle_weights') && numel(public_vars.particle_weights) == size(particles, 1)
    weights = public_vars.particle_weights(:);
end

if isempty(particles)
    estimated_pose = nan(1,3);
    return;
end

valid = all(isfinite(particles(:, 1:3)), 2);
particles = particles(valid, :);
if ~isempty(weights)
    weights = weights(valid);
end
if isempty(particles)
    estimated_pose = nan(1, 3);
    return;
end
if isempty(weights) || any(~isfinite(weights)) || sum(weights) <= 0
    weights = ones(size(particles, 1), 1) / size(particles, 1);
else
    weights = weights / sum(weights);
end

xy = particles(:, 1:2);
N = size(xy, 1);
if N <= 8
    cluster = particles;
    cluster_w = weights;
else
    % Use the densest local neighborhood instead of the global mean.
    dx = xy(:, 1) - xy(:, 1).';
    dy = xy(:, 2) - xy(:, 2).';
    d2 = dx.^2 + dy.^2;
    support_radius2 = 0.75^2;
    local_support = (d2 <= support_radius2) * weights;
    [~, center_idx] = max(local_support);

    cluster_mask = d2(center_idx, :) <= 0.85^2;
    if nnz(cluster_mask) < max(12, ceil(0.08 * N))
        [~, order] = sort(d2(center_idx, :), 'ascend');
        keep_n = min(max(20, ceil(0.12 * N)), N);
        cluster_mask = false(1, N);
        cluster_mask(order(1:keep_n)) = true;
    end
    cluster = particles(cluster_mask, :);
    cluster_w = weights(cluster_mask);
end
cluster_w = cluster_w / max(sum(cluster_w), 1e-12);

estimated_pose = zeros(1, 3);
estimated_pose(1:2) = sum(cluster(:, 1:2) .* cluster_w, 1);
estimated_pose(3) = atan2(sum(cluster_w .* sin(cluster(:, 3))), sum(cluster_w .* cos(cluster(:, 3))));

end

