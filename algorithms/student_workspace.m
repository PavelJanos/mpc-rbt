function [public_vars] = student_workspace(read_only_vars, public_vars)
%STUDENT_WORKSPACE Main project orchestration loop.
%
% Keep this file as a high-level state machine. Put algorithms into their
% dedicated folders and call them from here.

if read_only_vars.counter == 1
    public_vars = init_project_workspace(read_only_vars, public_vars);
end

public_vars.counter = read_only_vars.counter;
gnss_available = is_gnss_available(read_only_vars);
public_vars = bootstrap_particle_filter_from_gnss(read_only_vars, public_vars, gnss_available);

% 1. Localization update
[public_vars.particles, public_vars.particle_weights, public_vars.pf_stats] = ...
    update_particle_filter(read_only_vars, public_vars);
public_vars = apply_rejected_pose_penalty(read_only_vars, public_vars);
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
if gnss_available && isfield(public_vars, 'mu') && numel(public_vars.mu) >= 3 ...
        && all(isfinite(public_vars.mu(1:3)))
    pf_pose = estimate_pose(public_vars);
    theta = public_vars.mu(3);
    if numel(pf_pose) >= 3 && isfinite(pf_pose(3))
        theta = pf_pose(3);
    end
    public_vars.estimated_pose = [public_vars.mu(1), public_vars.mu(2), theta];
else
    public_vars.estimated_pose = estimate_pose(public_vars);
end

% 2. Situation checks
localized = is_localized(public_vars);
obstacle_distance = min_lidar_distance(read_only_vars);
enter_escape = obstacle_distance < 0.22;
exit_escape = obstacle_distance > 0.30;
has_path = isfield(public_vars, 'path') && ~isempty(public_vars.path) && size(public_vars.path, 1) >= 2;
public_vars = update_false_goal_detector(read_only_vars, public_vars);
public_vars = update_stuck_detector(public_vars);

% 3. Main state machine
switch string(public_vars.mode)
    case "localize"
        if gnss_available
            public_vars.mode = "plan";
            public_vars.replan_path = true;
            public_vars.align_before_drive = true;
            public_vars.motion_vector = [0, 0];
        else
            public_vars.motion_vector = localization_motion(read_only_vars);
        end

        if localized && ~gnss_available
            public_vars.mode = "plan";
            public_vars.replan_path = true;
            public_vars.align_before_drive = true;
            public_vars.motion_vector = [0, 0];
        end

    case "plan"
        [planned_path, raw_planner_path] = plan_path(read_only_vars, public_vars);
        public_vars.path = planned_path;
        public_vars.smoothed_path = planned_path;
        public_vars.display_path = planned_path;
        public_vars.global_path = planned_path;
        public_vars.raw_path = planned_path;
        public_vars.raw_planner_path = raw_planner_path;
        public_vars.replan_path = false;
        public_vars.path_revision = public_vars.path_revision + 1;

        if isempty(public_vars.path) || size(public_vars.path, 1) < 2
            public_vars.mode = "localize";
            public_vars.motion_vector = [0, 0];
        elseif isfield(public_vars, 'align_before_drive') && public_vars.align_before_drive
            public_vars.mode = "align";
            public_vars.align_before_drive = false;
            public_vars.motion_vector = [0, 0];
        else
            public_vars.mode = "drive";
            public_vars.motion_vector = [0, 0];
        end

    case "align"
        if enter_escape
            public_vars.mode = "escape";
            public_vars.motion_vector = escape_motion(read_only_vars);
        elseif ~localized
            public_vars.mode = "localize";
            public_vars.replan_path = true;
            public_vars.motion_vector = localization_motion(read_only_vars);
        elseif ~has_path
            public_vars.mode = "plan";
            public_vars.replan_path = true;
            public_vars.motion_vector = [0, 0];
        else
            [aligned, public_vars.motion_vector] = align_to_path_motion(read_only_vars, public_vars);
            if aligned
                public_vars.mode = "drive";
                public_vars.motion_vector = [0, 0];
            end
        end

    case "drive"
        if enter_escape
            public_vars.mode = "escape";
            public_vars.motion_vector = escape_motion(read_only_vars);
        elseif ~localized
            public_vars.mode = "localize";
            public_vars.replan_path = true;
            public_vars.motion_vector = localization_motion(read_only_vars);
        elseif ~has_path
            public_vars.mode = "plan";
            public_vars.replan_path = true;
            public_vars.motion_vector = [0, 0];
        else
            public_vars = plan_motion(read_only_vars, public_vars);
        end

    case "escape"
        public_vars.motion_vector = escape_motion(read_only_vars);

        if exit_escape
            public_vars.mode = "plan";
            public_vars.replan_path = true;
            public_vars.align_before_drive = true;
        end

    otherwise
        public_vars.mode = "localize";
        public_vars.motion_vector = [0, 0];
end

end

function public_vars = init_project_workspace(read_only_vars, public_vars)
public_vars.motion_vector = [0, 0];
public_vars.mode = "localize";
public_vars.path = [];
public_vars.smoothed_path = [];
public_vars.display_path = [];
public_vars.global_path = [];
public_vars.raw_path = [];
public_vars.raw_planner_path = [];
public_vars.replan_path = true;
public_vars.align_before_drive = false;
public_vars.near_goal_counter = 0;
public_vars.false_goal_radius_m = 0.45;
public_vars.false_goal_steps = 35;
public_vars.rejected_poses = zeros(0, 3);
public_vars.rejected_pose_expire_steps = zeros(0, 1);
public_vars.rejected_pose_ttl_steps = 250;
public_vars.rejected_pose_radius_m = 0.80;
public_vars.rejected_pose_weight_scale = 0.01;
public_vars.pf_gnss_initialized = false;
public_vars.pf_gnss_init_samples = zeros(0, 2);
public_vars.pf_gnss_init_samples_needed = 8;
public_vars.pf_gnss_init_spread_m = 0.40;
public_vars.pf_gnss_init_safe_margin_m = 0.12;
public_vars.stuck_pose_history = zeros(0, 2);
public_vars.stuck_window_steps = 25;
public_vars.stuck_move_threshold_m = 0.18;
public_vars.stuck_detected = false;
public_vars.path_revision = 0;
public_vars.estimated_pose = nan(1, 3);
public_vars.path_planner_mode = 'astar';
public_vars.path_smoothing_mode = 'chaikin';
public_vars.smooth_chaikin_iters = 5;
public_vars.controller_mode = 'pure_pursuit';
public_vars.skip_start_alignment = true;

public_vars = init_particle_filter(read_only_vars, public_vars);
public_vars = init_kalman_filter(read_only_vars, public_vars);
end

function localized = is_localized(public_vars)
localized = false;

if ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || any(~isfinite(public_vars.estimated_pose(1:3)))
    return;
end

if isfield(public_vars, 'pf_stats') && ~isempty(public_vars.pf_stats)
    if isfield(public_vars.pf_stats, 'dominant_radius_m') ...
            && isfield(public_vars.pf_stats, 'dominant_mass') ...
            && isfinite(public_vars.pf_stats.dominant_radius_m) ...
            && isfinite(public_vars.pf_stats.dominant_mass)
        localized = public_vars.pf_stats.dominant_radius_m < 0.45 ...
            && public_vars.pf_stats.dominant_mass > 0.58;
        return;
    end
end

if isfield(public_vars, 'sigma') && ~isempty(public_vars.sigma) ...
        && all(isfinite(public_vars.sigma(:)))
    localized = mean([public_vars.sigma(1, 1), public_vars.sigma(2, 2)]) < 0.35;
end
end

function public_vars = update_stuck_detector(public_vars)
public_vars.stuck_detected = false;

if ~isfield(public_vars, 'stuck_pose_history') || isempty(public_vars.stuck_pose_history)
    public_vars.stuck_pose_history = zeros(0, 2);
end
if ~isfield(public_vars, 'stuck_window_steps') || isempty(public_vars.stuck_window_steps)
    public_vars.stuck_window_steps = 25;
end
if ~isfield(public_vars, 'stuck_move_threshold_m') || isempty(public_vars.stuck_move_threshold_m)
    public_vars.stuck_move_threshold_m = 0.18;
end
if ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || numel(public_vars.estimated_pose) < 2 || any(~isfinite(public_vars.estimated_pose(1:2)))
    public_vars.stuck_pose_history = zeros(0, 2);
    return;
end

public_vars.stuck_pose_history(end + 1, :) = public_vars.estimated_pose(1:2);
if size(public_vars.stuck_pose_history, 1) > public_vars.stuck_window_steps
    public_vars.stuck_pose_history = public_vars.stuck_pose_history(end - public_vars.stuck_window_steps + 1:end, :);
end

stuck_modes = ["drive", "align", "escape"];
if ~any(string(public_vars.mode) == stuck_modes) ...
        || size(public_vars.stuck_pose_history, 1) < public_vars.stuck_window_steps
    return;
end

move_dist = norm(public_vars.stuck_pose_history(end, :) - public_vars.stuck_pose_history(1, :));
if move_dist < public_vars.stuck_move_threshold_m
    public_vars.stuck_detected = true;
    public_vars.mode = "escape";
    public_vars.replan_path = true;
    public_vars.align_before_drive = true;
    public_vars.stuck_pose_history = zeros(0, 2);
end
end

function available = is_gnss_available(read_only_vars)
available = false;

if ~isfield(read_only_vars, 'gnss_position') || isempty(read_only_vars.gnss_position)
    return;
end

gnss_xy = read_only_vars.gnss_position(1:min(2, numel(read_only_vars.gnss_position)));
available = numel(gnss_xy) == 2 && all(isfinite(gnss_xy));
end

function public_vars = bootstrap_particle_filter_from_gnss(read_only_vars, public_vars, gnss_available)
if ~gnss_available
    return;
end
if ~isfield(public_vars, 'pf_gnss_initialized') || isempty(public_vars.pf_gnss_initialized)
    public_vars.pf_gnss_initialized = false;
end
if public_vars.pf_gnss_initialized
    return;
end
if ~isfield(public_vars, 'pf_gnss_init_samples') || isempty(public_vars.pf_gnss_init_samples)
    public_vars.pf_gnss_init_samples = zeros(0, 2);
end
if ~isfield(public_vars, 'pf_gnss_init_samples_needed') || isempty(public_vars.pf_gnss_init_samples_needed)
    public_vars.pf_gnss_init_samples_needed = 8;
end
if ~isfield(public_vars, 'pf_gnss_init_spread_m') || isempty(public_vars.pf_gnss_init_spread_m)
    public_vars.pf_gnss_init_spread_m = 0.40;
end
if ~isfield(public_vars, 'pf_gnss_init_safe_margin_m') || isempty(public_vars.pf_gnss_init_safe_margin_m)
    public_vars.pf_gnss_init_safe_margin_m = 0.12;
end

public_vars.pf_gnss_init_samples(end + 1, :) = read_only_vars.gnss_position(1:2);
if size(public_vars.pf_gnss_init_samples, 1) < public_vars.pf_gnss_init_samples_needed
    return;
end

center_xy = mean(public_vars.pf_gnss_init_samples, 1);
if ~isfield(public_vars, 'particles') || isempty(public_vars.particles)
    n_particles = read_only_vars.max_particles;
else
    n_particles = size(public_vars.particles, 1);
end
n_particles = max(1, min(n_particles, read_only_vars.max_particles));

particles = sample_particles_near_gnss(read_only_vars.map, center_xy, n_particles, ...
    public_vars.pf_gnss_init_spread_m, public_vars.pf_gnss_init_safe_margin_m);
public_vars.particles = particles;
public_vars.particle_weights = ones(size(particles, 1), 1) / max(size(particles, 1), 1);
public_vars.pf_gnss_initialized = true;
end

function particles = sample_particles_near_gnss(map, center_xy, n_particles, spread_m, safe_margin_m)
particles = zeros(n_particles, 3);
filled = 0;
tries = 0;
max_tries = max(1000, 200 * n_particles);
limits = map.limits;

while filled < n_particles && tries < max_tries
    tries = tries + 1;
    xy = center_xy + spread_m * randn(1, 2);
    xy(1) = max(limits(1), min(limits(3), xy(1)));
    xy(2) = max(limits(2), min(limits(4), xy(2)));
    if point_too_close_to_wall(xy, map.walls, safe_margin_m)
        continue;
    end
    filled = filled + 1;
    particles(filled, :) = [xy, -pi + 2 * pi * rand()];
end

if filled < n_particles
    particles = particles(1:max(filled, 1), :);
    if filled == 0
        particles(1, :) = [center_xy, -pi + 2 * pi * rand()];
    end
end
end

function tf = point_too_close_to_wall(point, walls, margin)
tf = false;
for i = 1:size(walls, 1)
    if point_to_segment_distance_local(point, walls(i, 1:2), walls(i, 3:4)) <= margin
        tf = true;
        return;
    end
end
end

function public_vars = update_false_goal_detector(read_only_vars, public_vars)
if ~isfield(public_vars, 'near_goal_counter') || isempty(public_vars.near_goal_counter)
    public_vars.near_goal_counter = 0;
end
if ~isfield(public_vars, 'false_goal_radius_m') || isempty(public_vars.false_goal_radius_m)
    public_vars.false_goal_radius_m = 0.45;
end
if ~isfield(public_vars, 'false_goal_steps') || isempty(public_vars.false_goal_steps)
    public_vars.false_goal_steps = 35;
end

if ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || numel(public_vars.estimated_pose) < 2 || any(~isfinite(public_vars.estimated_pose(1:2))) ...
        || ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars.map, 'goal')
    public_vars.near_goal_counter = 0;
    return;
end

goal_xy = read_only_vars.map.goal(1:2);
estimated_goal_dist = norm(public_vars.estimated_pose(1:2) - goal_xy);
if estimated_goal_dist < public_vars.false_goal_radius_m
    public_vars.near_goal_counter = public_vars.near_goal_counter + 1;
else
    public_vars.near_goal_counter = 0;
end

if public_vars.near_goal_counter < public_vars.false_goal_steps
    return;
end

public_vars = remember_rejected_pose(read_only_vars, public_vars, public_vars.estimated_pose);
public_vars = init_particle_filter(read_only_vars, public_vars);
public_vars = init_kalman_filter(read_only_vars, public_vars);
public_vars.path = [];
public_vars.smoothed_path = [];
public_vars.display_path = [];
public_vars.global_path = [];
public_vars.raw_path = [];
public_vars.raw_planner_path = [];
public_vars.replan_path = true;
public_vars.align_before_drive = false;
public_vars.path_revision = public_vars.path_revision + 1;
public_vars.mode = "localize";
public_vars.motion_vector = [0, 0];
public_vars.near_goal_counter = 0;
end

function public_vars = remember_rejected_pose(read_only_vars, public_vars, pose)
if isempty(pose) || numel(pose) < 3 || any(~isfinite(pose(1:3)))
    return;
end

if ~isfield(public_vars, 'rejected_poses') || isempty(public_vars.rejected_poses)
    public_vars.rejected_poses = zeros(0, 3);
end
if ~isfield(public_vars, 'rejected_pose_expire_steps') || isempty(public_vars.rejected_pose_expire_steps)
    public_vars.rejected_pose_expire_steps = zeros(0, 1);
end
if ~isfield(public_vars, 'rejected_pose_ttl_steps') || isempty(public_vars.rejected_pose_ttl_steps)
    public_vars.rejected_pose_ttl_steps = 250;
end

expire_step = read_only_vars.counter + public_vars.rejected_pose_ttl_steps;
public_vars.rejected_poses(end + 1, :) = pose(1:3);
public_vars.rejected_pose_expire_steps(end + 1, 1) = expire_step;
public_vars = prune_rejected_poses(read_only_vars, public_vars);
end

function public_vars = apply_rejected_pose_penalty(read_only_vars, public_vars)
public_vars = prune_rejected_poses(read_only_vars, public_vars);

if ~isfield(public_vars, 'rejected_poses') || isempty(public_vars.rejected_poses) ...
        || ~isfield(public_vars, 'particles') || isempty(public_vars.particles)
    return;
end

if ~isfield(public_vars, 'particle_weights') ...
        || numel(public_vars.particle_weights) ~= size(public_vars.particles, 1)
    public_vars.particle_weights = ones(size(public_vars.particles, 1), 1) / max(size(public_vars.particles, 1), 1);
end
if ~isfield(public_vars, 'rejected_pose_radius_m') || isempty(public_vars.rejected_pose_radius_m)
    public_vars.rejected_pose_radius_m = 0.80;
end
if ~isfield(public_vars, 'rejected_pose_weight_scale') || isempty(public_vars.rejected_pose_weight_scale)
    public_vars.rejected_pose_weight_scale = 0.01;
end

particles_xy = public_vars.particles(:, 1:2);
penalized = false(size(public_vars.particles, 1), 1);
for i = 1:size(public_vars.rejected_poses, 1)
    rejected_xy = public_vars.rejected_poses(i, 1:2);
    if any(~isfinite(rejected_xy))
        continue;
    end
    dist = vecnorm(particles_xy - rejected_xy, 2, 2);
    penalized = penalized | dist < public_vars.rejected_pose_radius_m;
end

if any(penalized)
    public_vars.particle_weights(penalized) = ...
        public_vars.particle_weights(penalized) * public_vars.rejected_pose_weight_scale;
    weight_sum = sum(public_vars.particle_weights);
    if isfinite(weight_sum) && weight_sum > 0
        public_vars.particle_weights = public_vars.particle_weights / weight_sum;
    else
        public_vars.particle_weights = ones(size(public_vars.particles, 1), 1) / max(size(public_vars.particles, 1), 1);
    end
end
end

function public_vars = prune_rejected_poses(read_only_vars, public_vars)
if ~isfield(public_vars, 'rejected_poses') || isempty(public_vars.rejected_poses) ...
        || ~isfield(public_vars, 'rejected_pose_expire_steps') || isempty(public_vars.rejected_pose_expire_steps)
    return;
end

keep = public_vars.rejected_pose_expire_steps(:) > read_only_vars.counter;
public_vars.rejected_poses = public_vars.rejected_poses(keep, :);
public_vars.rejected_pose_expire_steps = public_vars.rejected_pose_expire_steps(keep);
end

function distance = min_lidar_distance(read_only_vars)
distance = inf;

if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances)
    return;
end

d = read_only_vars.lidar_distances(:);
d = d(isfinite(d));
if isempty(d)
    return;
end

distance = min(d);
end

function motion_vector = localization_motion(read_only_vars)
max_vel = read_only_vars.agent_drive.max_vel;
turn_speed = min(0.28, max_vel);
motion_vector = [turn_speed, -turn_speed];
end

function motion_vector = escape_motion(read_only_vars)
motion_vector = [0, 0];

if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances) ...
        || ~isfield(read_only_vars, 'lidar_config') || isempty(read_only_vars.lidar_config)
    return;
end

d = read_only_vars.lidar_distances(:);
a = read_only_vars.lidar_config(:);
n = min(numel(d), numel(a));
d = d(1:n);
a = a(1:n);

valid = isfinite(d) & isfinite(a);
if ~any(valid)
    return;
end

score = -inf(n, 1);
for i = 1:n
    if ~valid(i)
        continue;
    end
    prev_i = i - 1;
    next_i = i + 1;
    if prev_i < 1
        prev_i = n;
    end
    if next_i > n
        next_i = 1;
    end

    left_d = neighbor_distance_or_zero(d, valid, prev_i);
    right_d = neighbor_distance_or_zero(d, valid, next_i);
    score(i) = d(i) + 0.5 * left_d + 0.5 * right_d;
end

[~, best_i] = max(score);
angle_error = wrap_to_pi_local(a(best_i));
max_vel = read_only_vars.agent_drive.max_vel;

if abs(angle_error) > 0.65
    turn_speed = min(0.40, max_vel);
    motion_vector = [turn_speed * sign(angle_error), -turn_speed * sign(angle_error)];
else
    forward_speed = min(0.35, max_vel);
    turn_gain = 0.80;
    vR = forward_speed + turn_gain * angle_error;
    vL = forward_speed - turn_gain * angle_error;
    motion_vector = clamp_wheel_speeds([vR, vL], max_vel);
end
end

function [aligned, motion_vector] = align_to_path_motion(read_only_vars, public_vars)
aligned = false;
motion_vector = [0, 0];

if ~isfield(public_vars, 'path') || isempty(public_vars.path) || size(public_vars.path, 1) < 2
    aligned = true;
    return;
end
if ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || numel(public_vars.estimated_pose) < 3 || any(~isfinite(public_vars.estimated_pose(1:3)))
    return;
end

path = public_vars.path;
pose = public_vars.estimated_pose;
seg_idx = nearest_path_segment(path, pose(1:2));
target_heading = atan2(path(seg_idx + 1, 2) - path(seg_idx, 2), ...
    path(seg_idx + 1, 1) - path(seg_idx, 1));
heading_error = wrap_to_pi_local(target_heading - pose(3));

align_tolerance = 15 * pi / 180;
if abs(heading_error) <= align_tolerance
    aligned = true;
    return;
end

max_vel = read_only_vars.agent_drive.max_vel;
turn_speed = min(0.25, max_vel);
motion_vector = [turn_speed * sign(heading_error), -turn_speed * sign(heading_error)];
end

function seg_idx = nearest_path_segment(path, point)
seg_idx = 1;
best_dist = inf;

for i = 1:(size(path, 1) - 1)
    d = point_to_segment_distance_local(point, path(i, :), path(i + 1, :));
    if d < best_dist
        best_dist = d;
        seg_idx = i;
    end
end
end

function d = point_to_segment_distance_local(point, a, b)
ab = b - a;
den = dot(ab, ab);
if den < 1e-12
    d = norm(point - a);
    return;
end
t = dot(point - a, ab) / den;
t = max(0, min(1, t));
projection = a + t * ab;
d = norm(point - projection);
end

function value = neighbor_distance_or_zero(d, valid, idx)
if valid(idx)
    value = d(idx);
else
    value = 0;
end
end

function speeds = clamp_wheel_speeds(speeds, max_vel)
speeds = max(min(speeds, max_vel), -max_vel);
end

function angle = wrap_to_pi_local(angle)
angle = mod(angle + pi, 2 * pi) - pi;
end
