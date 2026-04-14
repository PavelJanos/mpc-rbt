function [particles, weights, pf_stats] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here

particles = public_vars.particles;
if isfield(public_vars, 'particle_weights') && numel(public_vars.particle_weights) == size(particles, 1)
    weights = public_vars.particle_weights(:);
else
    weights = ones(size(particles, 1), 1) / max(size(particles, 1), 1);
end
pf_stats = struct('ess', nan, 'best_weight', nan, 'second_weight', nan, ...
    'top_ratio', nan, 'dominant_mass', nan, 'dominant_radius_m', nan, ...
    'hypothesis_count', nan, 'top_hypotheses', struct([]));

% I. Prediction
for i=1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% II. Correction
measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
for i=1:size(particles, 1)
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end
likelihood = weight_particles(measurements, read_only_vars.lidar_distances);
if isempty(weights) || numel(weights) ~= numel(likelihood) || any(~isfinite(weights)) || sum(weights) <= 0
    weights = ones(size(likelihood)) / max(numel(likelihood), 1);
end
weights = weights .* likelihood;
weights = weights / max(sum(weights), 1e-12);

ambiguity_active = isfield(public_vars, 'environment_ambiguity') ...
    && isfield(public_vars.environment_ambiguity, 'active') ...
    && public_vars.environment_ambiguity.active;
post_reseed_hold = isfield(public_vars, 'localize') ...
    && isfield(public_vars.localize, 'post_reseed_hold_counter') ...
    && public_vars.localize.post_reseed_hold_counter > 0;
if ambiguity_active || post_reseed_hold
    % Keep multiple hypotheses alive longer in symmetric corridors.
    uniform_w = ones(size(weights)) / max(numel(weights), 1);
    weights = weights .^ 0.82;
    weights = weights / max(sum(weights), 1e-12);
    mix_uniform = 0.30;
    if post_reseed_hold
        mix_uniform = 0.45;
    end
    weights = (1 - mix_uniform) * weights + mix_uniform * uniform_w;
    weights = weights / max(sum(weights), 1e-12);
end

pf_stats = compute_pf_stats(read_only_vars, public_vars, particles, weights);
single_mode_collapse_hold = post_reseed_hold ...
    && isfinite(getfield(pf_stats, 'hypothesis_count')) && pf_stats.hypothesis_count <= 1 ...
    && isfinite(getfield(pf_stats, 'top_ratio')) && pf_stats.top_ratio < 1.20;

% III. Resampling
relocalize_mode = isfield(public_vars, 'nav_state') ...
    && any(strcmp(string(public_vars.nav_state), ["relocalize", "disambiguate", "localize"]));
ess_ratio = 0.65;
rough_xy = 0.03;
rough_th = 0.05;
inject_fraction = 0.12;
if relocalize_mode
    ess_ratio = 0.45;
    rough_xy = 0.07;
    rough_th = 0.10;
    inject_fraction = 0.22;
end
if ambiguity_active
    ess_ratio = min(ess_ratio, 0.35);
    rough_xy = max(rough_xy, 0.09);
    rough_th = max(rough_th, 0.14);
    inject_fraction = max(inject_fraction, 0.30);
end
if post_reseed_hold
    ess_ratio = min(ess_ratio, 0.25);
    rough_xy = max(rough_xy, 0.10);
    rough_th = max(rough_th, 0.16);
    inject_fraction = max(inject_fraction, 0.36);
end
if single_mode_collapse_hold
    ess_ratio = min(ess_ratio, 0.18);
    rough_xy = max(rough_xy, 0.14);
    rough_th = max(rough_th, 0.22);
    inject_fraction = max(inject_fraction, 0.50);
end

if isfinite(pf_stats.ess) && pf_stats.ess < ess_ratio * size(particles, 1)
    [particles, idx] = resample_particles_with_idx(particles, weights);
    particles = roughen_particles(particles, read_only_vars.map, rough_xy, rough_th);
    weights = ones(size(idx)) / max(numel(idx), 1);
end
particles = inject_random_particles(particles, read_only_vars.map, inject_fraction);
if numel(weights) ~= size(particles, 1)
    weights = ones(size(particles, 1), 1) / max(size(particles, 1), 1);
else
    weights = weights / max(sum(weights), 1e-12);
end


end

function particles = inject_random_particles(particles, map, fraction)
N = size(particles, 1);
if N == 0 || fraction <= 0
    return;
end

replace_n = max(1, round(fraction * N));
limits = map.limits;
safe_margin = 0.12;
max_tries = 50000;
filled = 0;
tries = 0;

while filled < replace_n && tries < max_tries
    tries = tries + 1;
    x = limits(1) + (limits(3) - limits(1)) * rand();
    y = limits(2) + (limits(4) - limits(2)) * rand();
    th = -pi + 2 * pi * rand();
    if is_collision_with_walls_local([x, y], map.walls, safe_margin)
        continue;
    end
    filled = filled + 1;
    particles(end - replace_n + filled, :) = [x, y, th];
end
end

function particles = roughen_particles(particles, map, sigma_xy, sigma_th)
if isempty(particles)
    return;
end
limits = map.limits;
for i = 1:size(particles, 1)
    particles(i, 1) = min(max(particles(i, 1) + sigma_xy * randn(), limits(1)), limits(3));
    particles(i, 2) = min(max(particles(i, 2) + sigma_xy * randn(), limits(2)), limits(4));
    particles(i, 3) = wrap_to_pi_local_pf(particles(i, 3) + sigma_th * randn());
end
end

function [new_particles, idx] = resample_particles_with_idx(particles, weights)
method = 'systematic';
N = size(particles, 1);
w = weights(:);
if numel(w) ~= N || any(~isfinite(w)) || sum(w) <= 0
    w = ones(N, 1) / N;
else
    w = w / sum(w);
end

switch lower(method)
    case 'multinomial'
        idx = multinomial_resample_idx_local(w, N);
    case 'stratified'
        idx = stratified_resample_idx_local(w, N);
    case 'residual'
        idx = residual_resample_idx_local(w, N);
    otherwise
        idx = systematic_resample_idx_local(w, N);
end
new_particles = particles(idx, :);
end

function idx = multinomial_resample_idx_local(w, N)
cdf = cumsum(w);
u = rand(N, 1);
idx = arrayfun(@(x) find(cdf >= x, 1, 'first'), u);
end

function idx = systematic_resample_idx_local(w, N)
cdf = cumsum(w);
u0 = rand() / N;
u = u0 + (0:(N - 1))' / N;
idx = zeros(N, 1);
j = 1;
for i = 1:N
    while u(i) > cdf(j)
        j = j + 1;
    end
    idx(i) = j;
end
end

function idx = stratified_resample_idx_local(w, N)
cdf = cumsum(w);
u = ((0:(N - 1))' + rand(N, 1)) / N;
idx = zeros(N, 1);
j = 1;
for i = 1:N
    while u(i) > cdf(j)
        j = j + 1;
    end
    idx(i) = j;
end
end

function idx = residual_resample_idx_local(w, N)
det_count = floor(N * w);
idx = repelem((1:N)', det_count);
R = N - numel(idx);
if R > 0
    res_w = N * w - det_count;
    s = sum(res_w);
    if s <= 0
        tail = randi(N, R, 1);
    else
        res_w = res_w / s;
        tail = multinomial_resample_idx_local(res_w, R);
    end
    idx = [idx; tail];
end
idx = idx(randperm(N));
end

function tf = is_collision_with_walls_local(point, walls, tol)
tf = false;
for i = 1:size(walls, 1)
    a = walls(i, 1:2);
    b = walls(i, 3:4);
    ab = b - a;
    if all(abs(ab) < 1e-12)
        d = norm(point - a);
    else
        t = dot(point - a, ab) / dot(ab, ab);
        t = max(0, min(1, t));
        p = a + t * ab;
        d = norm(point - p);
    end
    if d <= tol
        tf = true;
        return;
    end
end
end

function pf_stats = compute_pf_stats(read_only_vars, public_vars, particles, weights)
pf_stats = struct('ess', nan, 'best_weight', nan, 'second_weight', nan, ...
    'top_ratio', nan, 'dominant_mass', nan, 'dominant_radius_m', nan, ...
    'hypothesis_count', nan, 'top_hypotheses', struct([]));
if isempty(particles) || isempty(weights)
    return;
end

w = weights(:);
N = numel(w);
if N == 0 || any(~isfinite(w)) || sum(w) <= 0
    return;
end
w = w / sum(w);
pf_stats.ess = 1 / sum(w.^2);

[w_sorted, idx_sorted] = sort(w, 'descend');
pf_stats.best_weight = w_sorted(1);
if numel(w_sorted) >= 2
    pf_stats.second_weight = w_sorted(2);
    pf_stats.top_ratio = w_sorted(1) / max(w_sorted(2), 1e-12);
else
    pf_stats.second_weight = nan;
    pf_stats.top_ratio = inf;
end

best_xy = particles(idx_sorted(1), 1:2);
d = vecnorm(particles(:, 1:2) - best_xy, 2, 2);
dominant_mask = d <= 0.65;
pf_stats.dominant_mass = sum(w(dominant_mask));
pf_stats.dominant_radius_m = sum(w .* d);
pf_stats.hypothesis_count = count_weighted_hypotheses(particles(:, 1:2), w, 0.55, 0.05);
pf_stats.top_hypotheses = extract_top_hypotheses(read_only_vars, public_vars, particles, w, 3, 0.50, 0.04);
end

function hypotheses = extract_top_hypotheses(read_only_vars, public_vars, particles, w, max_keep, cluster_radius_m, min_mass)
hypotheses = struct('pose', {}, 'mass', {}, 'radius_m', {}, 'scan_score', {}, 'age', {}, 'stability', {});
if isempty(particles) || isempty(w)
    return;
end

[~, order] = sort(w, 'descend');
remaining = true(numel(w), 1);
prev_hyp = struct([]);
if isfield(public_vars, 'pf_stats') && isstruct(public_vars.pf_stats) ...
        && isfield(public_vars.pf_stats, 'top_hypotheses') && ~isempty(public_vars.pf_stats.top_hypotheses)
    prev_hyp = public_vars.pf_stats.top_hypotheses;
end

for ii = 1:numel(order)
    idx = order(ii);
    if ~remaining(idx)
        continue;
    end
    d = vecnorm(particles(:, 1:2) - particles(idx, 1:2), 2, 2);
    mask = remaining & (d <= cluster_radius_m);
    mass = sum(w(mask));
    remaining(mask) = false;
    if mass < min_mass
        continue;
    end

    cw = w(mask);
    cw = cw / max(sum(cw), 1e-12);
    pose = zeros(1, 3);
    pose(1:2) = sum(particles(mask, 1:2) .* cw, 1);
    pose(3) = atan2(sum(cw .* sin(particles(mask, 3))), sum(cw .* cos(particles(mask, 3))));
    radius_m = sum(cw .* vecnorm(particles(mask, 1:2) - pose(1:2), 2, 2));
    scan_score = compute_hypothesis_scan_score(read_only_vars, pose);
    [age, stability] = match_previous_hypothesis(prev_hyp, pose, mass, radius_m, scan_score);

    hypotheses(end + 1).pose = pose; %#ok<AGROW>
    hypotheses(end).mass = mass;
    hypotheses(end).radius_m = radius_m;
    hypotheses(end).scan_score = scan_score;
    hypotheses(end).age = age;
    hypotheses(end).stability = stability;
    if numel(hypotheses) >= max_keep
        break;
    end
end
end

function scan_score = compute_hypothesis_scan_score(read_only_vars, pose)
scan_score = 0;
if ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars, 'lidar_config') ...
        || ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances) ...
        || ~any(isfinite(read_only_vars.lidar_distances))
    return;
end
pred = compute_lidar_measurement(read_only_vars.map, pose, read_only_vars.lidar_config);
cost = lidar_match_cost(pred, read_only_vars.lidar_distances);
if isfinite(cost)
    scan_score = max(0, 1 - cost / 0.35);
end
end

function [age, stability] = match_previous_hypothesis(prev_hyp, pose, mass, radius_m, scan_score)
age = 1;
stability = 0.45 * mass + 0.30 * scan_score + 0.25 * max(0, 1 - radius_m / 0.60);
if isempty(prev_hyp)
    return;
end

best_idx = 0;
best_cost = inf;
for i = 1:numel(prev_hyp)
    if ~isfield(prev_hyp(i), 'pose') || numel(prev_hyp(i).pose) < 3
        continue;
    end
    dxy = norm(prev_hyp(i).pose(1:2) - pose(1:2));
    dth = abs(wrap_to_pi_local_pf(prev_hyp(i).pose(3) - pose(3)));
    cost = dxy + 0.35 * dth;
    if cost < best_cost
        best_cost = cost;
        best_idx = i;
    end
end
if best_idx > 0 && best_cost <= 1.10
    prev_age = 0;
    prev_stability = 0;
    if isfield(prev_hyp(best_idx), 'age') && isfinite(prev_hyp(best_idx).age)
        prev_age = prev_hyp(best_idx).age;
    end
    if isfield(prev_hyp(best_idx), 'stability') && isfinite(prev_hyp(best_idx).stability)
        prev_stability = prev_hyp(best_idx).stability;
    end
    age = prev_age + 1;
    stability = 0.65 * prev_stability + 0.35 * stability;
end
end

function n = count_weighted_hypotheses(xy, w, cluster_radius_m, min_mass)
n = 0;
if isempty(xy) || isempty(w)
    return;
end
w = w(:);
w = w / max(sum(w), 1e-12);
[~, order] = sort(w, 'descend');
remaining = true(numel(w), 1);
for ii = 1:numel(order)
    idx = order(ii);
    if ~remaining(idx)
        continue;
    end
    d = vecnorm(xy - xy(idx, :), 2, 2);
    mask = remaining & (d <= cluster_radius_m);
    mass = sum(w(mask));
    remaining(mask) = false;
    if mass >= min_mass
        n = n + 1;
    end
end
if n == 0
    n = 1;
end
end

function a = wrap_to_pi_local_pf(a)
a = mod(a + pi, 2 * pi) - pi;
end

function cost = lidar_match_cost(pred, meas)
cost = inf;
if isempty(pred) || isempty(meas)
    return;
end
pred = pred(:);
meas = meas(:);
mask = isfinite(pred) & isfinite(meas);
if nnz(mask) < 8
    return;
end
err = abs(pred(mask) - meas(mask));
cost = median(err);
end

