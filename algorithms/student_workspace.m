function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    % Initialize data collection for Task 2
    public_vars.lidar_history = [];
    public_vars.gnss_history = [];
    if ~isfield(public_vars, 'path_planner_mode') || isempty(public_vars.path_planner_mode)
        public_vars.path_planner_mode = 'dijkstra';
    end
    if ~isfield(public_vars, 'force_grid_planner')
        public_vars.force_grid_planner = false;
    end
    if ~isfield(public_vars, 'replan_path')
        public_vars.replan_path = true;
    end
    if ~isfield(public_vars, 'path_smoothing_mode') || isempty(public_vars.path_smoothing_mode)
        public_vars.path_smoothing_mode = 'iterative';
    end
    if ~isfield(public_vars, 'path_clearance_m') || ~isfinite(public_vars.path_clearance_m)
        public_vars.path_clearance_m = 0.2;
    end

    % Indoor localization warm-up (no MoCap): first stabilize pose estimate,
    % then start full path tracking.
    public_vars.warmup.enabled = true;
    public_vars.warmup.done = false;
    public_vars.warmup.min_steps = 8;
    public_vars.warmup.max_steps = 45;
    public_vars.warmup.cluster_thresh_m = 0.45;
    public_vars.warmup.k_heading = 0.9;
    public_vars.warmup.w_max = 0.20;
    public_vars.warmup.heading_tol_rad = 10 * pi / 180;
    public_vars.warmup.turn_accum_rad = 0;
    public_vars.warmup.max_turn_rad = 0.55 * pi;
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

gnss_valid = isfield(read_only_vars, 'gnss_position') && ~isempty(read_only_vars.gnss_position) ...
    && all(isfinite(read_only_vars.gnss_position(1:2)));
lidar_valid = isfield(read_only_vars, 'lidar_distances') && ~isempty(read_only_vars.lidar_distances) ...
    && any(isfinite(read_only_vars.lidar_distances));

% 9. Update particle filter
if lidar_valid
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
pf_pose = estimate_pose(public_vars); % (x,y,theta)
pf_valid = ~isempty(pf_pose) && all(isfinite(pf_pose));
kf_valid = isfield(public_vars, 'kf_enabled') && public_vars.kf_enabled ...
    && isfield(public_vars, 'mu') && ~isempty(public_vars.mu) ...
    && all(isfinite(public_vars.mu(:)));

if kf_valid && pf_valid
    kf_pose = public_vars.mu(:)';
    [pf_var_xy, pf_var_th] = pf_uncertainty(public_vars);

    if isfield(public_vars, 'sigma') && ~isempty(public_vars.sigma) && all(isfinite(public_vars.sigma(:)))
        kf_var_xy = max([public_vars.sigma(1,1), public_vars.sigma(2,2)], 1e-6);
        kf_var_th = max(public_vars.sigma(3,3), 1e-6);
    else
        kf_var_xy = [1, 1];
        kf_var_th = 1;
    end

    % Robust gating: if estimates disagree too much, avoid blind blending.
    d_xy = norm(kf_pose(1:2) - pf_pose(1:2));
    d_th = abs(wrap_to_pi_ws(kf_pose(3) - pf_pose(3)));
    agree = (d_xy <= 1.2) && (d_th <= 100 * pi / 180);

    if agree
        [xy_fused, w_xy] = fuse_xy(kf_pose(1:2), kf_var_xy, pf_pose(1:2), pf_var_xy);
        [th_fused, w_th] = fuse_theta(kf_pose(3), kf_var_th, pf_pose(3), pf_var_th);
        public_vars.estimated_pose = [xy_fused, th_fused];
        public_vars.fusion.mode = 'blend';
        public_vars.fusion.weights_xy = w_xy;
        public_vars.fusion.weights_theta = w_th;
    else
        % Pick the estimate with lower reported XY uncertainty.
        kf_unc = mean(kf_var_xy);
        pf_unc = mean(pf_var_xy);
        if kf_unc <= pf_unc
            public_vars.estimated_pose = kf_pose;
            public_vars.fusion.mode = 'kf_selected';
        else
            public_vars.estimated_pose = pf_pose;
            public_vars.fusion.mode = 'pf_selected';
        end
    end
elseif kf_valid
    public_vars.estimated_pose = public_vars.mu(:)';
    public_vars.fusion.mode = 'kf_only';
elseif lidar_valid && pf_valid
    public_vars.estimated_pose = pf_pose;
    public_vars.fusion.mode = 'pf_only';
elseif pf_valid
    public_vars.estimated_pose = pf_pose;
    public_vars.fusion.mode = 'pf_only';
elseif ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose)
    public_vars.estimated_pose = [nan, nan, nan];
    public_vars.fusion.mode = 'none';
end

% Task 2: Collect sensor data
public_vars.lidar_history = [public_vars.lidar_history; read_only_vars.lidar_distances];
if ~isempty(read_only_vars.gnss_position)
    public_vars.gnss_history = [public_vars.gnss_history; read_only_vars.gnss_position];
end

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);
if isfield(public_vars, 'replan_path') && public_vars.replan_path
    public_vars.replan_path = false;
end

% 13. Plan next motion command
[run_warmup, public_vars] = should_run_warmup(read_only_vars, public_vars);
if run_warmup
    path_heading = initial_path_heading_ws(public_vars.path);
    if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose) ...
            && all(isfinite(public_vars.estimated_pose(1:3)))
        e = wrap_to_pi_ws(path_heading - public_vars.estimated_pose(3));
    else
        e = 0;
    end

    w = max(min(public_vars.warmup.k_heading * e, public_vars.warmup.w_max), -public_vars.warmup.w_max);
    L = read_only_vars.agent_drive.interwheel_dist;
    public_vars.motion_vector = [0.5 * L * w, -0.5 * L * w];

    L = read_only_vars.agent_drive.interwheel_dist;
    dt = read_only_vars.sampling_period;
    omega = (public_vars.motion_vector(1) - public_vars.motion_vector(2)) / max(L, 1e-6);
    public_vars.warmup.turn_accum_rad = public_vars.warmup.turn_accum_rad + abs(omega * dt);
    if abs(e) <= public_vars.warmup.heading_tol_rad && read_only_vars.counter >= public_vars.warmup.min_steps
        public_vars.warmup.done = true;
        public_vars.motion_vector = [0, 0];
        public_vars.skip_start_alignment = true;
    elseif public_vars.warmup.turn_accum_rad >= public_vars.warmup.max_turn_rad
        public_vars.warmup.done = true;
        public_vars.motion_vector = [0, 0];
        public_vars.skip_start_alignment = true;
    end
else
    public_vars = plan_motion(read_only_vars, public_vars);
end

function psi0 = initial_path_heading_ws(path)
if isempty(path) || size(path, 1) < 2
    psi0 = 0;
    return;
end
idx = 2;
while idx <= size(path, 1) && norm(path(idx, :) - path(1, :)) < 1e-6
    idx = idx + 1;
end
if idx > size(path, 1)
    psi0 = 0;
else
    d = path(idx, :) - path(1, :);
    psi0 = atan2(d(2), d(1));
end
end

function a = wrap_to_pi_ws(a)
a = mod(a + pi, 2 * pi) - pi;
end



end

function [tf, public_vars] = should_run_warmup(read_only_vars, public_vars)
tf = false;
if ~isfield(public_vars, 'warmup') || ~isfield(public_vars.warmup, 'enabled') || ~public_vars.warmup.enabled
    return;
end
if isfield(public_vars.warmup, 'done') && public_vars.warmup.done
    return;
end

gnss_valid = isfield(read_only_vars, 'gnss_position') && ~isempty(read_only_vars.gnss_position) ...
    && all(isfinite(read_only_vars.gnss_position(1:2)));
if gnss_valid
    public_vars.warmup.done = true;
    return;
end

if ~isfield(public_vars, 'particles') || isempty(public_vars.particles)
    tf = true;
    return;
end

xy = public_vars.particles(:, 1:2);
xy = xy(all(isfinite(xy), 2), :);
if isempty(xy)
    tf = true;
    return;
end

center = median(xy, 1);
cluster_radius = median(vecnorm(xy - center, 2, 2));

enough_steps = read_only_vars.counter >= public_vars.warmup.min_steps;
tight_cluster = cluster_radius <= public_vars.warmup.cluster_thresh_m;
timeout = read_only_vars.counter >= public_vars.warmup.max_steps;

if timeout || (enough_steps && tight_cluster)
    public_vars.warmup.done = true;
    tf = false;
else
    tf = true;
end
end

function [xy_fused, weights] = fuse_xy(kf_xy, kf_var_xy, pf_xy, pf_var_xy)
kf_var_xy = max(kf_var_xy(:)', 1e-6);
pf_var_xy = max(pf_var_xy(:)', 1e-6);
wk = 1 ./ kf_var_xy;
wp = 1 ./ pf_var_xy;
den = wk + wp;
xy_fused = (wk .* kf_xy + wp .* pf_xy) ./ den;
weights = [wk ./ den; wp ./ den]; % row1 EKF, row2 PF
end

function [th_fused, weights] = fuse_theta(kf_th, kf_var_th, pf_th, pf_var_th)
kf_var_th = max(kf_var_th, 1e-6);
pf_var_th = max(pf_var_th, 1e-6);
wk = 1 / kf_var_th;
wp = 1 / pf_var_th;
den = wk + wp;
wk_n = wk / den;
wp_n = wp / den;
v = wk_n * [cos(kf_th), sin(kf_th)] + wp_n * [cos(pf_th), sin(pf_th)];
th_fused = atan2(v(2), v(1));
weights = [wk_n, wp_n]; % [EKF, PF]
end

function [var_xy, var_th] = pf_uncertainty(public_vars)
var_xy = [1, 1];
var_th = 1;
if ~isfield(public_vars, 'particles') || isempty(public_vars.particles)
    return;
end
p = public_vars.particles;
valid = all(isfinite(p(:, 1:3)), 2);
p = p(valid, :);
if size(p, 1) < 5
    return;
end
vx = var(p(:, 1), 1);
vy = var(p(:, 2), 1);
var_xy = max([vx, vy], 1e-6);

% Circular variance: 1 - R, bounded (0..1)
c = mean(cos(p(:, 3)));
s = mean(sin(p(:, 3)));
R = sqrt(c^2 + s^2);
var_th = max(1 - R, 1e-3);
end

