function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Path-following control with switchable algorithms.
% Available modes (set in public_vars.controller_mode):
% 1) 'waypoint_p'
% 2) 'pure_pursuit'  (default)
% 3) 'cross_track_pd'
% 4) 'stanley'

persistent state;
if isempty(state) || read_only_vars.counter == 1
    state.wp_idx = 1;
    state.path_idx = 1;
    state.prev_cte = 0;
    state.last_valid_pose = [];
    state.start_heading_aligned = false;
    state.last_path_revision = -1;
    state.stuck_front_counter = 0;
    state.escape_turn_counter = 0;
    state.escape_turn_dir = 1;
    state.path_entry_mode = "align";
    state.pp_progress_idx = 1;
    state.pp_last_look_idx = 1;
end

if ~isfield(public_vars, 'controller_mode') || isempty(public_vars.controller_mode)
    public_vars.controller_mode = 'pure_pursuit';
end

public_vars.motion_debug.controller_mode = string(public_vars.controller_mode);
public_vars.motion_debug.align_active = false;
public_vars.motion_debug.align_target_heading = nan;
public_vars.motion_debug.align_heading_error = nan;
public_vars.motion_debug.nearest_idx = nan;
public_vars.motion_debug.look_idx = nan;
public_vars.motion_debug.target_x = nan;
public_vars.motion_debug.target_y = nan;
public_vars.motion_debug.alpha = nan;
public_vars.motion_debug.curvature = nan;
public_vars.motion_debug.lookahead_dist = nan;

path = public_vars.path;
if isempty(path) || size(path, 1) < 2
    public_vars.motion_vector = [0, 0];
    return;
end

path_revision = -1;
if isfield(public_vars, 'path_revision') && ~isempty(public_vars.path_revision) ...
        && isfinite(public_vars.path_revision)
    path_revision = public_vars.path_revision;
end
new_path_revision = path_revision ~= state.last_path_revision;
if path_revision ~= state.last_path_revision
    state.wp_idx = 1;
    state.path_idx = 1;
    state.prev_cte = 0;
    state.last_path_revision = path_revision;
    state.stuck_front_counter = 0;
    state.escape_turn_counter = 0;
    state.path_entry_mode = "align";
    state.pp_progress_idx = 1;
    state.pp_last_look_idx = 1;
end

pose = get_pose(read_only_vars, public_vars);
if isempty(pose) || numel(pose) < 3 || any(~isfinite(pose(1:3)))
    if ~isempty(state.last_valid_pose) && all(isfinite(state.last_valid_pose(1:3)))
        pose = state.last_valid_pose;
    else
        public_vars.motion_vector = [0, 0];
        return;
    end
else
    state.last_valid_pose = pose;
end
if isempty(pose)
    public_vars.motion_vector = [0, 0];
    return;
end

track_commit_active = isfield(public_vars, 'track') && isfield(public_vars.track, 'commit_counter') ...
    && isfinite(public_vars.track.commit_counter) && public_vars.track.commit_counter > 0;

drive = read_only_vars.agent_drive;
Ts = read_only_vars.sampling_period;
goal = path(end, :);
goal_dist = norm(pose(1:2) - goal);
nav_state = "";
if isfield(public_vars, 'nav_state') && ~isempty(public_vars.nav_state)
    nav_state = string(public_vars.nav_state);
end

if new_path_revision
    [start_seg_idx, ~] = nearest_path_idx(path, pose(1:2), 1);
    start_seg_idx = min(max(start_seg_idx, 1), max(1, size(path, 1) - 1));
    near_path_start = norm(pose(1:2) - path(start_seg_idx, :)) <= 0.65;
    target_heading = atan2(path(start_seg_idx + 1, 2) - path(start_seg_idx, 2), ...
        path(start_seg_idx + 1, 1) - path(start_seg_idx, 1));
    heading_jump = abs(wrap_to_pi(target_heading - pose(3)));
    if ~track_commit_active && near_path_start && heading_jump >= 45 * pi / 180
        state.start_heading_aligned = false;
    end
end

% Pre-drive orientation stage:
% before moving, rotate in place if heading is strongly misaligned with the
% first path segment direction.
if isfield(public_vars, 'skip_start_alignment') && public_vars.skip_start_alignment
    state.start_heading_aligned = true;
end

% Path entry has its own dedicated align/merge controller. If the generic
% start-heading stage remains active here, it can keep the robot in
% rotation-only mode and prevent any actual merge progress.
if nav_state == "path_entry"
    state.start_heading_aligned = true;
end

if ~state.start_heading_aligned
    [align_seg_idx, ~] = nearest_path_idx(path, pose(1:2), 1);
    align_seg_idx = min(max(align_seg_idx, 1), max(1, size(path, 1) - 1));
    near_path_start = norm(pose(1:2) - path(align_seg_idx, :)) <= 0.9;
    if ~near_path_start
        state.start_heading_aligned = true;
    end
end

if ~state.start_heading_aligned
    [align_seg_idx, ~] = nearest_path_idx(path, pose(1:2), 1);
    align_seg_idx = min(max(align_seg_idx, 1), max(1, size(path, 1) - 1));
    target_heading = atan2(path(align_seg_idx + 1, 2) - path(align_seg_idx, 2), ...
        path(align_seg_idx + 1, 1) - path(align_seg_idx, 1));
    e0 = wrap_to_pi(target_heading - pose(3));
    align_tol = 12 * pi / 180;     % [rad]
    release_tol = 6 * pi / 180;    % [rad], hysteresis
    k_align = 2.4;
    w_align_max = 1.15;
    w_align = max(min(k_align * e0, w_align_max), -w_align_max);
    public_vars.motion_debug.align_active = true;
    public_vars.motion_debug.align_target_heading = target_heading;
    public_vars.motion_debug.align_heading_error = e0;
    public_vars.motion_debug.target_x = path(min(2, size(path, 1)), 1);
    public_vars.motion_debug.target_y = path(min(2, size(path, 1)), 2);

    if abs(e0) > align_tol
        public_vars.motion_debug.base_v = 0;
        public_vars.motion_debug.base_w = w_align;
        public_vars.motion_vector = vw_to_wheels(0, w_align, drive);
        return;
    end
    if abs(e0) <= release_tol
        state.start_heading_aligned = true;
    end
end

if nav_state == "path_entry"
    [v, w, state, ctrl_dbg] = ctrl_path_entry(pose, path, state);
    dbg_fields = fieldnames(ctrl_dbg);
    for i = 1:numel(dbg_fields)
        public_vars.motion_debug.(dbg_fields{i}) = ctrl_dbg.(dbg_fields{i});
    end
    public_vars.motion_debug.base_v = v;
    public_vars.motion_debug.base_w = w;
    public_vars.motion_debug.v_after_lidar = v;
    public_vars.motion_debug.v_after_loc = v;
    public_vars.motion_debug.v_after_pathdist = v;
    public_vars.motion_debug.v_after_map = v;
    public_vars.motion_debug.v_after_preview = v;
    public_vars.motion_debug.limiter = "none";

    [v, w] = apply_goal_approach_safety(v, w, goal_dist);
    public_vars.motion_debug.base_v = v;
    public_vars.motion_debug.base_w = w;
    public_vars.motion_debug.v_after_lidar = v;
    public_vars.motion_debug.v_after_loc = v;
    public_vars.motion_debug.v_after_pathdist = v;
    public_vars.motion_debug.v_after_map = v;
    public_vars.motion_debug.v_after_preview = v;

    [v, w] = apply_lidar_safety(v, w, read_only_vars, public_vars);
    public_vars.motion_debug.v_after_lidar = v;
    [v, w] = apply_localization_safety(v, w, public_vars, read_only_vars);
    public_vars.motion_debug.v_after_loc = v;
    [v, w] = apply_path_distance_safety(v, w, public_vars);
    public_vars.motion_debug.v_after_pathdist = v;
    [v, w] = apply_map_safety(v, w, pose, read_only_vars, public_vars);
    public_vars.motion_debug.v_after_map = v;
    [v, w] = apply_path_preview_safety(v, w, pose, path, state, read_only_vars, public_vars);
    public_vars.motion_debug.v_after_preview = v;

    [~, idx_lim] = min([public_vars.motion_debug.v_after_lidar, ...
        public_vars.motion_debug.v_after_loc, ...
        public_vars.motion_debug.v_after_pathdist, ...
        public_vars.motion_debug.v_after_map, ...
        public_vars.motion_debug.v_after_preview]);
    lim_names = ["lidar", "localization", "path_distance", "map", "preview"];
    if isfinite(v) && isfinite(public_vars.motion_debug.base_v) && v < public_vars.motion_debug.base_v - 1e-6
        public_vars.motion_debug.limiter = lim_names(idx_lim);
    end

    [v, w, state, escaped] = apply_stuck_front_escape(v, w, pose, path, state, read_only_vars, public_vars);
    if escaped
        public_vars.motion_debug.limiter = "stuck_escape";
    end

    if ~isfinite(v) || ~isfinite(w)
        public_vars.motion_vector = [0, 0];
        return;
    end

    public_vars.motion_vector = vw_to_wheels(v, w, drive);
    return;
end

switch lower(public_vars.controller_mode)
    case 'waypoint_p'
        [v, w, state, ctrl_dbg] = ctrl_waypoint_p(pose, path, state);
    case 'pure_pursuit'
    [v, w, state, ctrl_dbg] = ctrl_pure_pursuit(pose, path, state, read_only_vars, public_vars);
    case 'cross_track_pd'
        [v, w, state, ctrl_dbg] = ctrl_cross_track_pd(pose, path, Ts, state);
    case 'stanley'
        [v, w, state, ctrl_dbg] = ctrl_stanley(pose, path, state);
    otherwise
        [v, w, state, ctrl_dbg] = ctrl_pure_pursuit(pose, path, state, read_only_vars, public_vars);
end

dbg_fields = fieldnames(ctrl_dbg);
for i = 1:numel(dbg_fields)
    public_vars.motion_debug.(dbg_fields{i}) = ctrl_dbg.(dbg_fields{i});
end

public_vars.motion_debug.base_v = v;
public_vars.motion_debug.base_w = w;
public_vars.motion_debug.v_after_lidar = v;
public_vars.motion_debug.v_after_loc = v;
public_vars.motion_debug.v_after_pathdist = v;
public_vars.motion_debug.v_after_map = v;
public_vars.motion_debug.v_after_preview = v;
public_vars.motion_debug.limiter = "none";

[v, w] = apply_goal_approach_safety(v, w, goal_dist);
public_vars.motion_debug.base_v = v;
public_vars.motion_debug.base_w = w;
public_vars.motion_debug.v_after_lidar = v;
public_vars.motion_debug.v_after_loc = v;
public_vars.motion_debug.v_after_pathdist = v;
public_vars.motion_debug.v_after_map = v;
public_vars.motion_debug.v_after_preview = v;

[v, w] = apply_lidar_safety(v, w, read_only_vars, public_vars);
public_vars.motion_debug.v_after_lidar = v;
[v, w] = apply_localization_safety(v, w, public_vars, read_only_vars);
public_vars.motion_debug.v_after_loc = v;
[v, w] = apply_path_distance_safety(v, w, public_vars);
public_vars.motion_debug.v_after_pathdist = v;
[v, w] = apply_map_safety(v, w, pose, read_only_vars, public_vars);
public_vars.motion_debug.v_after_map = v;
[v, w] = apply_path_preview_safety(v, w, pose, path, state, read_only_vars, public_vars);
public_vars.motion_debug.v_after_preview = v;

[~, idx_lim] = min([public_vars.motion_debug.v_after_lidar, ...
    public_vars.motion_debug.v_after_loc, ...
    public_vars.motion_debug.v_after_pathdist, ...
    public_vars.motion_debug.v_after_map, ...
    public_vars.motion_debug.v_after_preview]);
lim_names = ["lidar", "localization", "path_distance", "map", "preview"];
if isfinite(v) && isfinite(public_vars.motion_debug.base_v) && v < public_vars.motion_debug.base_v - 1e-6
    public_vars.motion_debug.limiter = lim_names(idx_lim);
end

[v, w, state, escaped] = apply_stuck_front_escape(v, w, pose, path, state, read_only_vars, public_vars);
if escaped
    public_vars.motion_debug.limiter = "stuck_escape";
end

if ~isfinite(v) || ~isfinite(w)
    public_vars.motion_vector = [0, 0];
    return;
end

public_vars.motion_vector = vw_to_wheels(v, w, drive);

end

function [v_out, w_out, state, escaped] = apply_stuck_front_escape(v, w, pose, path, state, read_only_vars, public_vars)
v_out = v;
w_out = w;
escaped = false;

if ~isfield(public_vars, 'nav_state') || ~any(strcmp(string(public_vars.nav_state), ["track", "path_entry"]))
    state.stuck_front_counter = 0;
    state.escape_turn_counter = 0;
    return;
end
if numel(pose) < 3 || any(~isfinite(pose(1:3))) || size(path, 1) < 2
    state.stuck_front_counter = 0;
    state.escape_turn_counter = 0;
    return;
end

[front_min, left_min, right_min] = lidar_sector_minima_pm(read_only_vars);
goal_dist = norm(pose(1:2) - path(end, :));
base_v = nan;
if isfield(public_vars, 'motion_debug') && isfield(public_vars.motion_debug, 'base_v')
    base_v = public_vars.motion_debug.base_v;
end
path_dist = inf;
if isfield(public_vars, 'localization_quality') && isfield(public_vars.localization_quality, 'path_distance_m') ...
        && isfinite(public_vars.localization_quality.path_distance_m)
    path_dist = public_vars.localization_quality.path_distance_m;
end

in_path_entry = isfield(public_vars, 'nav_state') && strcmp(string(public_vars.nav_state), "path_entry");
front_blocked = isfinite(front_min) && front_min < 0.48;
barely_moving = isfinite(v_out) && abs(v_out) < 0.05;
wants_forward = isfinite(base_v) && base_v > 0.16;
not_near_goal = isfinite(goal_dist) && goal_dist > 0.9;
off_path = isfinite(path_dist) && path_dist > 0.08;
if in_path_entry
    wants_forward = isfinite(base_v) && base_v > 0.10;
    off_path = isfinite(path_dist) && path_dist > 0.12;
end

if front_blocked && barely_moving && wants_forward && not_near_goal && off_path
    state.stuck_front_counter = state.stuck_front_counter + 1;
else
    state.stuck_front_counter = max(0, state.stuck_front_counter - 1);
end

if state.escape_turn_counter > 0
    state.escape_turn_counter = state.escape_turn_counter - 1;
    v_out = 0;
    w_out = 1.1 * state.escape_turn_dir;
    escaped = true;
    return;
end

if state.stuck_front_counter >= 5
    if isfinite(left_min) && isfinite(right_min)
        if right_min > left_min + 0.08
            state.escape_turn_dir = -1;
        elseif left_min > right_min + 0.08
            state.escape_turn_dir = 1;
        else
            state.escape_turn_dir = sign(w_out);
            if state.escape_turn_dir == 0
                state.escape_turn_dir = 1;
            end
        end
    else
        state.escape_turn_dir = 1;
    end
    state.escape_turn_counter = 14;
    state.stuck_front_counter = 0;
    v_out = 0;
    if in_path_entry
        w_out = 1.25 * state.escape_turn_dir;
    else
        w_out = 1.1 * state.escape_turn_dir;
    end
    escaped = true;
end
end

function pose = get_pose(read_only_vars, public_vars)
pose = [];

if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose)
    pose = public_vars.estimated_pose;
    return;
end

% No MoCap fallback by design.
end

function [v, w, state, dbg] = ctrl_waypoint_p(pose, path, state)
K_heading = 2.5;
v_nom = 0.55;
wp_tol = 0.25;
dbg = struct('nearest_idx', nan, 'look_idx', nan, 'target_x', nan, 'target_y', nan, ...
    'alpha', nan, 'curvature', nan, 'lookahead_dist', nan);

idx = state.wp_idx;
idx = min(max(idx, 1), size(path, 1));

while idx < size(path, 1) && norm(path(idx, :) - pose(1:2)) < wp_tol
    idx = idx + 1;
end
state.wp_idx = idx;

target = path(idx, :);
target_heading = atan2(target(2) - pose(2), target(1) - pose(1));
e_heading = wrap_to_pi(target_heading - pose(3));
dbg.nearest_idx = idx;
dbg.look_idx = idx;
dbg.target_x = target(1);
dbg.target_y = target(2);
dbg.alpha = e_heading;

v = v_nom * (1 - 0.5 * min(abs(e_heading) / pi, 1));
w = K_heading * e_heading;
end

function [v, w, state, dbg] = ctrl_path_entry(pose, path, state)
v = 0;
w = 0;
dbg = struct('nearest_idx', nan, 'look_idx', nan, 'target_x', nan, 'target_y', nan, ...
    'alpha', nan, 'curvature', nan, 'lookahead_dist', nan);

[seg_idx, e_ct, psi_path] = nearest_segment_error(path, pose(1:2), max(min(state.path_idx, size(path, 1) - 1), 1));
seg_idx = min(max(seg_idx, 1), size(path, 1) - 1);
state.path_idx = seg_idx;

target = path(seg_idx + 1, :);
heading_err = wrap_to_pi(psi_path - pose(3));
dbg.nearest_idx = seg_idx;
dbg.look_idx = seg_idx + 1;
dbg.target_x = target(1);
dbg.target_y = target(2);
dbg.alpha = heading_err;
dbg.curvature = e_ct;
dbg.lookahead_dist = norm(target - pose(1:2));

% Path entry must be a stable two-phase merge:
% 1) align to the nearest path segment heading
% 2) only then drive forward with mild cross-track correction
align_enter = 18 * pi / 180;
align_exit = 7 * pi / 180;
merge_realign = 20 * pi / 180;
w_align_max = 1.05;
w_merge_max = 0.8;
k_align = 2.2;
k_merge_heading = 1.5;
k_merge_ct = 0.45;

if ~isfield(state, 'path_entry_mode') || ~(state.path_entry_mode == "align" || state.path_entry_mode == "merge")
    state.path_entry_mode = "align";
end

if state.path_entry_mode == "merge" && abs(heading_err) > merge_realign
    state.path_entry_mode = "align";
elseif state.path_entry_mode == "align" && abs(heading_err) <= align_exit
    state.path_entry_mode = "merge";
end

if state.path_entry_mode == "align" || abs(heading_err) > align_enter
    state.path_entry_mode = "align";
    w = max(min(k_align * heading_err, w_align_max), -w_align_max);
    v = 0;
else
    state.path_entry_mode = "merge";
    w_merge = k_merge_heading * heading_err - k_merge_ct * e_ct;
    w = max(min(w_merge, w_merge_max), -w_merge_max);
    if abs(heading_err) > 10 * pi / 180 || abs(e_ct) > 0.12
        v = 0.08;
    else
        v = 0.18;
    end
end
end

function [v, w, state, dbg] = ctrl_pure_pursuit(pose, path, state, read_only_vars, public_vars)
v_nom = 0.46;
lookahead_nom = 0.68;
k_curve_slow = 0.95;
goal_capture_dist = 0.9;
w_max = 2.8;
k_recover_heading = 2.0;
k_recover_ct = 1.2;
lookahead_min = 0.24;
dbg = struct('nearest_idx', nan, 'look_idx', nan, 'target_x', nan, 'target_y', nan, ...
    'alpha', nan, 'curvature', nan, 'lookahead_dist', lookahead_nom);

[seg_idx, e_ct, psi_path] = nearest_segment_error(path, pose(1:2), max(min(state.path_idx, size(path, 1) - 1), 1));
nearest_idx = max(seg_idx, min(max(state.path_idx, 1), size(path, 1) - 1));
progress_idx = 1;
if isstruct(state) && isfield(state, 'pp_progress_idx') && ~isempty(state.pp_progress_idx) && isfinite(state.pp_progress_idx)
    progress_idx = state.pp_progress_idx;
end
progress_idx = max(min(progress_idx, size(path, 1) - 1), 1);
nearest_idx = max(nearest_idx, progress_idx);
state.path_idx = nearest_idx;
state.pp_progress_idx = nearest_idx;

e_heading_path = wrap_to_pi(psi_path - pose(3));
[preview_heading, preview_curvature] = preview_path_shape(path, nearest_idx);
e_heading_preview = wrap_to_pi(preview_heading - pose(3));
goal_dist = norm(path(end, :) - pose(1:2));
path_dist_hint = inf;
if nargin >= 5 && isfield(public_vars, 'localization_quality') ...
        && isfield(public_vars.localization_quality, 'path_distance_m') ...
        && isfinite(public_vars.localization_quality.path_distance_m)
    path_dist_hint = public_vars.localization_quality.path_distance_m;
end
front_min = inf;
left_min = inf;
right_min = inf;
if nargin >= 4 && ~isempty(read_only_vars)
    [front_min, left_min, right_min] = lidar_sector_minima_pm(read_only_vars);
end
front_open = (~isfinite(front_min)) || front_min >= 1.10;
left_open = (~isfinite(left_min)) || left_min >= 0.85;
right_open = (~isfinite(right_min)) || right_min >= 0.85;
open_space = front_open && left_open && right_open;
open_recovery = open_space && goal_dist >= 3.0 && isfinite(path_dist_hint) && path_dist_hint <= 0.60;
lookahead_dist = lookahead_nom;
if abs(preview_curvature) > 0.95
    lookahead_dist = 0.28;
elseif abs(preview_curvature) > 0.55
    lookahead_dist = 0.36;
elseif abs(e_ct) > 0.14 || abs(e_heading_preview) > 0.35
    lookahead_dist = 0.34;
elseif abs(e_ct) > 0.08 || abs(e_heading_preview) > 0.20
    lookahead_dist = 0.44;
end
lookahead_dist = max(lookahead_min, min(lookahead_nom, lookahead_dist));

look_idx = nearest_idx;
while look_idx < size(path, 1) && norm(path(look_idx, :) - pose(1:2)) < lookahead_dist
    look_idx = look_idx + 1;
end
last_look_idx = nearest_idx;
if isstruct(state) && isfield(state, 'pp_last_look_idx') && ~isempty(state.pp_last_look_idx) && isfinite(state.pp_last_look_idx)
    last_look_idx = state.pp_last_look_idx;
end
look_idx = max(look_idx, min(last_look_idx, size(path, 1)));

% Never deliberately target a waypoint that is behind the robot.
while look_idx < size(path, 1)
    vec_to_target = path(look_idx, :) - pose(1:2);
    heading_vec = [cos(pose(3)), sin(pose(3))];
    if dot(vec_to_target, heading_vec) >= -0.02
        break;
    end
    look_idx = look_idx + 1;
end

% Do not jump to the final waypoint until the robot is already near the
% last segment / goal neighborhood.
if size(path, 1) >= 3 && look_idx >= size(path, 1)
    dist_to_goal = norm(path(end, :) - pose(1:2));
    dist_to_pregoal = norm(path(end - 1, :) - pose(1:2));
    if dist_to_goal > goal_capture_dist && dist_to_pregoal > lookahead_dist
        look_idx = size(path, 1) - 1;
    end
end

target = path(look_idx, :);
alpha = wrap_to_pi(atan2(target(2) - pose(2), target(1) - pose(1)) - pose(3));

% If the chosen point is still strongly behind the robot, advance further.
while look_idx < size(path, 1) && abs(alpha) > 100 * pi / 180
    look_idx = look_idx + 1;
    target = path(look_idx, :);
    alpha = wrap_to_pi(atan2(target(2) - pose(2), target(1) - pose(1)) - pose(3));
end
state.pp_last_look_idx = max(look_idx, nearest_idx);

Ld = max(norm(target - pose(1:2)), 1e-3);
curvature = 2 * sin(alpha) / Ld;
curvature = 0.65 * curvature + 0.35 * preview_curvature;
dbg.nearest_idx = nearest_idx;
dbg.look_idx = look_idx;
dbg.target_x = target(1);
dbg.target_y = target(2);
dbg.alpha = alpha;
dbg.curvature = curvature;
dbg.lookahead_dist = lookahead_dist;

% Pure pursuit alone is weak when the robot drifts far from the path.
% In that case, switch to a segment-heading recovery so it turns back
% decisively instead of creeping forward with a small angular command.
if abs(e_ct) > 0.22 || abs(alpha) > 42 * pi / 180 || abs(e_heading_preview) > 38 * pi / 180
    recover_heading = wrap_to_pi(0.55 * e_heading_path + 0.45 * e_heading_preview);
    recover_alpha = wrap_to_pi(recover_heading + atan2(-k_recover_ct * e_ct, 0.34));
    dbg.alpha = recover_alpha;
    dbg.curvature = recover_alpha;
    if open_recovery
        if abs(recover_alpha) > 100 * pi / 180 || abs(e_ct) > 0.28
            v = 0.08;
        elseif abs(recover_alpha) > 80 * pi / 180 || abs(e_ct) > 0.22
            v = 0.12;
        elseif abs(recover_alpha) > 60 * pi / 180
            v = 0.18;
        elseif abs(recover_alpha) > 35 * pi / 180 || abs(e_ct) > 0.14
            v = 0.24;
        else
            v = 0.28;
        end
    else
        if abs(recover_alpha) > 100 * pi / 180 || abs(e_ct) > 0.28
            % Near stop-turn-go for very sharp recovery cases.
            v = 0.01;
        elseif abs(recover_alpha) > 80 * pi / 180 || abs(e_ct) > 0.22
            v = 0.06;
        elseif abs(recover_alpha) > 60 * pi / 180
            v = 0.10;
        elseif abs(recover_alpha) > 35 * pi / 180 || abs(e_ct) > 0.14
            v = 0.18;
        else
            v = 0.22;
        end
    end
    w = max(min(k_recover_heading * recover_alpha, w_max), -w_max);
else
    v = v_nom / (1 + k_curve_slow * abs(curvature));
    if abs(e_ct) > 0.14
        v = min(v, 0.22);
    elseif abs(e_ct) > 0.08
        v = min(v, 0.28);
    end
    if abs(preview_curvature) > 0.85
        v = min(v, 0.12);
    elseif abs(preview_curvature) > 0.55
        v = min(v, 0.18);
    end
    if abs(alpha) < 10 * pi / 180 && abs(e_ct) < 0.04
        v = max(v, 0.16);
    end
    if goal_dist < 1.0 && abs(alpha) < 12 * pi / 180 && abs(e_ct) < 0.06 && abs(e_heading_preview) < 0.18
        v = max(v, 0.20);
    end
    w = v * curvature;
    w = max(min(w, w_max), -w_max);
end
end

function [v, w, state, dbg] = ctrl_cross_track_pd(pose, path, Ts, state)
K_cte = 0.65;
K_heading = 1.2;
K_d = 0.02;
v_nom = 0.5;
w_max = 1.4;
dbg = struct('nearest_idx', nan, 'look_idx', nan, 'target_x', nan, 'target_y', nan, ...
    'alpha', nan, 'curvature', nan, 'lookahead_dist', nan);

[seg_idx, e_ct, psi_path] = nearest_segment_error(path, pose(1:2), state.path_idx);
state.path_idx = seg_idx;

e_heading = wrap_to_pi(psi_path - pose(3));
de = (e_ct - state.prev_cte) / max(Ts, 1e-3);
state.prev_cte = e_ct;
dbg.nearest_idx = seg_idx;
dbg.look_idx = seg_idx + 1;
dbg.target_x = path(min(seg_idx + 1, size(path, 1)), 1);
dbg.target_y = path(min(seg_idx + 1, size(path, 1)), 2);
dbg.alpha = e_heading;

w = K_heading * e_heading - K_cte * e_ct - K_d * de;
v = v_nom * (1 - 0.45 * min(abs(e_heading) / pi, 1));
w = max(min(w, w_max), -w_max);
end

function [v, w, state, dbg] = ctrl_stanley(pose, path, state)
k_stanley = 0.85;
K_heading = 1.40;
K_ct = 1.05;
v_nom = 0.30;
v_soft = 0.12;
w_max = 1.9;
dbg = struct('nearest_idx', nan, 'look_idx', nan, 'target_x', nan, 'target_y', nan, ...
    'alpha', nan, 'curvature', nan, 'lookahead_dist', nan);

[seg_idx, e_ct, psi_path] = nearest_segment_error(path, pose(1:2), state.path_idx);
state.path_idx = seg_idx;

e_heading = wrap_to_pi(psi_path - pose(3));
stanley_term = atan2(-k_stanley * e_ct, max(v_nom, 0.06) + v_soft);
if abs(e_heading) > 65 * pi / 180
    % When the robot is badly misaligned, heading recovery must dominate.
    stanley_term = 0.30 * stanley_term;
elseif abs(e_heading) > 40 * pi / 180
    stanley_term = 0.55 * stanley_term;
end
dbg.nearest_idx = seg_idx;
dbg.look_idx = seg_idx + 1;
dbg.target_x = path(min(seg_idx + 1, size(path, 1)), 1);
dbg.target_y = path(min(seg_idx + 1, size(path, 1)), 2);
dbg.alpha = e_heading;
dbg.curvature = stanley_term;
w = K_heading * e_heading + K_ct * stanley_term;
w = max(min(w, w_max), -w_max);

v = v_nom;
if abs(e_ct) > 0.45
    v = min(v, 0.14);
elseif abs(e_ct) > 0.28
    v = min(v, 0.20);
elseif abs(e_ct) > 0.16
    v = min(v, 0.25);
end

v = v * (1 - 0.72 * min(abs(e_heading) / pi, 1));
if abs(e_heading) > 85 * pi / 180
    v = min(v, 0.05);
elseif abs(e_heading) > 55 * pi / 180
    v = min(v, 0.12);
end

if abs(stanley_term) > 0.55
    v = min(v, 0.13);
elseif abs(stanley_term) > 0.35
    v = min(v, 0.20);
elseif abs(stanley_term) > 0.20
    v = min(v, 0.26);
end
end

function uv = vw_to_wheels(v, w, drive)
if ~isfinite(v) || ~isfinite(w)
    uv = [0, 0];
    return;
end

L = drive.interwheel_dist;
vR = v + 0.5 * L * w;
vL = v - 0.5 * L * w;

lim = drive.max_vel;
vR = max(min(vR, lim), -lim);
vL = max(min(vL, lim), -lim);

uv = [vR, vL];
end

function [idx, d] = nearest_path_idx(path, point, last_idx)
n = size(path, 1);
start_idx = min(max(last_idx, 1), n);
cand = start_idx:n;
delta = path(cand, :) - point;
[d2, local_idx] = min(sum(delta.^2, 2));
idx = cand(local_idx);
d = sqrt(d2);
end

function [heading, curvature] = preview_path_shape(path, idx)
heading = 0;
curvature = 0;
if isempty(path) || size(path, 1) < 3
    return;
end
idx0 = max(1, min(size(path, 1) - 1, idx));
idx1 = min(size(path, 1), idx0 + 2);
idx2 = min(size(path, 1), idx0 + 5);
v1 = path(idx1, :) - path(idx0, :);
v2 = path(idx2, :) - path(idx1, :);
if norm(v1) > 1e-6
    heading = atan2(v1(2), v1(1));
else
    heading = atan2(v2(2), v2(1));
end
if norm(v1) > 1e-6 && norm(v2) > 1e-6
    dtheta = wrap_to_pi(atan2(v2(2), v2(1)) - atan2(v1(2), v1(1)));
    arc_len = max(norm(v1) + norm(v2), 1e-6);
    curvature = dtheta / arc_len;
end
end

function [seg_idx, e_ct, psi_path] = nearest_segment_error(path, point, last_idx)
n = size(path, 1);
seg_start = min(max(last_idx, 1), n - 1);
best_d = inf;
seg_idx = seg_start;
best_proj = path(seg_start, :);
best_tangent = [1, 0];

for i = seg_start:(n - 1)
    a = path(i, :);
    b = path(i + 1, :);
    ab = b - a;
    if norm(ab) < 1e-9
        continue;
    end
    t = dot(point - a, ab) / dot(ab, ab);
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(point - proj);
    if d < best_d
        best_d = d;
        seg_idx = i;
        best_proj = proj;
        best_tangent = ab / norm(ab);
    end
end

psi_path = atan2(best_tangent(2), best_tangent(1));
rel = point - best_proj;
left_normal = [-best_tangent(2), best_tangent(1)];
e_ct = dot(rel, left_normal);
end

function psi0 = initial_path_heading(path)
if size(path, 1) < 2
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

function a = wrap_to_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end

function [v_safe, w_safe] = apply_lidar_safety(v, w, read_only_vars, public_vars)
v_safe = v;
w_safe = w;

if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances) ...
        || ~isfield(read_only_vars, 'lidar_config') || isempty(read_only_vars.lidar_config)
    return;
end

dist = read_only_vars.lidar_distances(:);
ang = read_only_vars.lidar_config(:);
valid = isfinite(dist) & isfinite(ang);
dist = dist(valid);
ang = ang(valid);
if isempty(dist)
    return;
end

front_mask = abs(wrap_to_pi(ang)) <= 35 * pi / 180;
left_mask = ang > 20 * pi / 180 & ang < 160 * pi / 180;
right_mask = ang < -20 * pi / 180 & ang > -160 * pi / 180;

if ~any(front_mask)
    return;
end

front_min = min(dist(front_mask));
left_min = min_or_default(dist(left_mask), inf);
right_min = min_or_default(dist(right_mask), inf);

track_mode = isfield(public_vars, 'nav_state') && strcmp(public_vars.nav_state, 'track');
commit_active = isfield(public_vars, 'track') && isfield(public_vars.track, 'commit_counter') ...
    && isfinite(public_vars.track.commit_counter) && public_vars.track.commit_counter > 0;
loc_confident = false;
path_dist = inf;
if isfield(public_vars, 'localization_quality') && ~isempty(public_vars.localization_quality)
    q = public_vars.localization_quality;
    if isfield(q, 'pf_cluster_radius_m') && isfinite(q.pf_cluster_radius_m) ...
            && q.pf_cluster_radius_m <= 0.20
        loc_confident = true;
    end
    if isfield(q, 'path_distance_m') && isfinite(q.path_distance_m)
        path_dist = q.path_distance_m;
    end
end

trusted_track = track_mode && loc_confident && commit_active;
if trusted_track
    slow_dist = 0.52;
    stop_dist = 0.24;
    side_slow_dist = 0.32;
    side_stop_dist = 0.18;
else
    slow_dist = 0.70;
    stop_dist = 0.42;
    side_slow_dist = 0.40;
    side_stop_dist = 0.26;
end
turn_gain = 1.2;
turn_max = 1.4;
front_creep_dist = 0.16;
side_creep_dist = 0.08;

turning_to_safer_side = false;
if front_min <= stop_dist
    if left_min > right_min && w_safe > 0.10
        turning_to_safer_side = true;
    elseif right_min >= left_min && w_safe < -0.10
        turning_to_safer_side = true;
    end
end

if front_min <= stop_dist
    if trusted_track && front_min > front_creep_dist
        scale = max(0.18, (front_min - front_creep_dist) / max(stop_dist - front_creep_dist, 1e-6));
        v_safe = min(v_safe * scale, 0.06);
        if left_min > right_min
            w_safe = max(w_safe, min(turn_max, 0.45 + turn_gain * (1 - scale)));
        else
            w_safe = min(w_safe, -min(turn_max, 0.45 + turn_gain * (1 - scale)));
        end
    elseif track_mode && loc_confident && turning_to_safer_side
        v_safe = min(v_safe, 0.08);
        if left_min > right_min
            w_safe = max(w_safe, min(turn_max, turn_gain * 0.6));
        else
            w_safe = min(w_safe, -min(turn_max, turn_gain * 0.6));
        end
    else
        v_safe = 0;
        if left_min > right_min
            w_safe = min(turn_max, turn_gain * (1 + max(left_min - right_min, 0)));
        else
            w_safe = -min(turn_max, turn_gain * (1 + max(right_min - left_min, 0)));
        end
    end
    return;
end

if front_min < slow_dist && v_safe > 0
    scale = max(0.15, (front_min - stop_dist) / max(slow_dist - stop_dist, 1e-6));
    v_safe = v_safe * scale;
    if left_min > right_min
        w_safe = w_safe + 0.6 * (1 - scale);
    else
        w_safe = w_safe - 0.6 * (1 - scale);
    end
    w_safe = max(min(w_safe, turn_max), -turn_max);
end

side_min = min(left_min, right_min);
if side_min <= side_stop_dist && v_safe > 0
    if trusted_track && front_min > slow_dist && side_min > side_creep_dist
        v_safe = min(v_safe, 0.05);
        if left_min < right_min
            w_safe = -min(max(abs(w_safe), 0.45) + 0.35, turn_max);
        else
            w_safe = min(max(abs(w_safe), 0.45) + 0.35, turn_max);
        end
    else
        v_safe = 0;
        if left_min < right_min
            w_safe = -min(abs(w_safe) + 1.0, turn_max);
        else
            w_safe = min(abs(w_safe) + 1.0, turn_max);
        end
    end
    return;
end

if side_min < side_slow_dist && v_safe > 0
    side_scale = max(0.18, (side_min - side_stop_dist) / max(side_slow_dist - side_stop_dist, 1e-6));
    v_safe = v_safe * side_scale;
    if left_min < right_min
        w_safe = w_safe - 0.65 * (1 - side_scale);
    else
        w_safe = w_safe + 0.65 * (1 - side_scale);
    end
    w_safe = max(min(w_safe, turn_max), -turn_max);
end
end

function [front_min, left_min, right_min] = lidar_sector_minima_pm(read_only_vars)
front_min = inf;
left_min = inf;
right_min = inf;
if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances) ...
        || ~isfield(read_only_vars, 'lidar_config') || isempty(read_only_vars.lidar_config)
    return;
end
dist = read_only_vars.lidar_distances(:);
ang = read_only_vars.lidar_config(:);
valid = isfinite(dist) & isfinite(ang);
dist = dist(valid);
ang = ang(valid);
if isempty(dist)
    return;
end
front_mask = abs(wrap_to_pi(ang)) <= 35 * pi / 180;
left_mask = ang > 20 * pi / 180 & ang < 160 * pi / 180;
right_mask = ang < -20 * pi / 180 & ang > -160 * pi / 180;
if any(front_mask), front_min = min(dist(front_mask)); end
if any(left_mask), left_min = min(dist(left_mask)); end
if any(right_mask), right_min = min(dist(right_mask)); end
end

function v = min_or_default(x, fallback)
if isempty(x)
    v = fallback;
else
    v = min(x);
end
end

function [v_safe, w_safe] = apply_localization_safety(v, w, public_vars, read_only_vars)
v_safe = v;
w_safe = w;

if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end

q = public_vars.localization_quality;
track_mode = isfield(public_vars, 'nav_state') && strcmp(public_vars.nav_state, 'track');
commit_active = isfield(public_vars, 'track') && isfield(public_vars.track, 'commit_counter') ...
    && isfinite(public_vars.track.commit_counter) && public_vars.track.commit_counter > 0;
pf_compact = isfield(q, 'pf_cluster_radius_m') && isfinite(q.pf_cluster_radius_m) ...
    && q.pf_cluster_radius_m <= 0.18;
pf_unique = isfield(q, 'pf_dominant_mass') && isfinite(q.pf_dominant_mass) ...
    && q.pf_dominant_mass >= 0.75;
trusted_track = track_mode && commit_active && pf_compact && pf_unique;
path_dist = inf;
if isfield(q, 'path_distance_m') && isfinite(q.path_distance_m)
    path_dist = q.path_distance_m;
end
front_min = inf;
left_min = inf;
right_min = inf;
if nargin >= 4 && ~isempty(read_only_vars)
    [front_min, left_min, right_min] = lidar_sector_minima_pm(read_only_vars);
end
front_open = (~isfinite(front_min)) || front_min >= 1.10;
left_open = (~isfinite(left_min)) || left_min >= 0.85;
right_open = (~isfinite(right_min)) || right_min >= 0.85;
open_space = front_open && left_open && right_open;
path_attached = isfinite(path_dist) && path_dist <= 0.20;
open_track = track_mode && path_attached && open_space;

if isfield(q, 'stop_translation') && q.stop_translation
    v_safe = 0;
end

if isfield(q, 'speed_scale') && isfinite(q.speed_scale)
    speed_scale = max(0, min(1, q.speed_scale));
    if trusted_track
        speed_scale = max(speed_scale, 0.78);
    elseif open_track
        speed_scale = max(speed_scale, 0.55);
    end
    v_safe = v_safe * speed_scale;
end

if isfield(q, 'turn_scale') && isfinite(q.turn_scale)
    turn_scale = max(0.2, min(1, q.turn_scale));
    if trusted_track
        turn_scale = max(turn_scale, 0.88);
    elseif open_track
        turn_scale = max(turn_scale, 0.82);
    end
    w_safe = w_safe * turn_scale;
end

if isfield(q, 'disagreement_theta_rad') && isfinite(q.disagreement_theta_rad)
    if (trusted_track || open_track) && q.disagreement_theta_rad <= 70 * pi / 180
        % In committed track, let the path follower keep turning authority
        % unless there is a very strong disagreement.
        return;
    elseif q.disagreement_theta_rad > 70 * pi / 180
        if open_track
            v_safe = min(v_safe, 0.18);
        else
            v_safe = min(v_safe, 0.10);
        end
    elseif q.disagreement_theta_rad > 40 * pi / 180
        if open_track
            v_safe = min(v_safe, 0.24);
        else
            v_safe = min(v_safe, 0.18);
        end
    end
end
end

function [v_safe, w_safe] = apply_map_safety(v, w, pose, read_only_vars, public_vars)
v_safe = v;
w_safe = w;

if ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars.map, 'walls') || isempty(read_only_vars.map.walls)
    return;
end
if numel(pose) < 3 || any(~isfinite(pose(1:3)))
    return;
end

warn_clear = 0.42;
stop_clear = 0.26;
nose_offset = 0.16;
boundary_warn = 0.34;
boundary_stop = 0.18;

track_mode = isfield(public_vars, 'nav_state') && strcmp(public_vars.nav_state, 'track');
commit_active = isfield(public_vars, 'track') && isfield(public_vars.track, 'commit_counter') ...
    && isfinite(public_vars.track.commit_counter) && public_vars.track.commit_counter > 0;
loc_confident = false;
path_dist = inf;
pf_unique = false;
if isfield(public_vars, 'localization_quality') && ~isempty(public_vars.localization_quality)
    q = public_vars.localization_quality;
    if isfield(q, 'pf_cluster_radius_m') && isfinite(q.pf_cluster_radius_m) ...
            && q.pf_cluster_radius_m <= 0.18
        loc_confident = true;
    end
    if isfield(q, 'pf_dominant_mass') && isfinite(q.pf_dominant_mass) ...
            && q.pf_dominant_mass >= 0.75
        pf_unique = true;
    end
    if isfield(q, 'path_distance_m') && isfinite(q.path_distance_m)
        path_dist = q.path_distance_m;
    end
end
trusted_track = track_mode && commit_active && loc_confident && pf_unique;
if trusted_track && isfinite(path_dist) && path_dist <= 0.35
    warn_clear = 0.22;
    stop_clear = 0.12;
    boundary_warn = 0.24;
    boundary_stop = 0.12;
elseif track_mode && loc_confident && isfinite(path_dist) && path_dist <= 0.18
    warn_clear = 0.30;
    stop_clear = 0.18;
    boundary_warn = 0.28;
    boundary_stop = 0.16;
end

walls = read_only_vars.map.walls;
[d_pose, closest_pose] = min_wall_distance(pose(1:2), walls);
nose = pose(1:2) + nose_offset * [cos(pose(3)), sin(pose(3))];
[d_nose, closest_nose] = min_wall_distance(nose, walls);

d_eff = d_pose;
closest_pt = closest_pose;
if d_nose < d_pose
    d_eff = d_nose;
    closest_pt = closest_nose;
end

if ~isfinite(d_eff)
    return;
end

if isfield(read_only_vars.map, 'limits') && numel(read_only_vars.map.limits) >= 4 ...
        && all(isfinite(read_only_vars.map.limits(1:4)))
    lim = read_only_vars.map.limits(:)';
    dist_left = nose(1) - lim(1);
    dist_right = lim(3) - nose(1);
    dist_bottom = nose(2) - lim(2);
    dist_top = lim(4) - nose(2);
    [boundary_d, boundary_idx] = min([dist_left, dist_right, dist_bottom, dist_top]);
    if boundary_d < boundary_warn
        switch boundary_idx
            case 1
                desired_heading = 0;
            case 2
                desired_heading = pi;
            case 3
                desired_heading = pi / 2;
            otherwise
                desired_heading = -pi / 2;
        end
        boundary_heading_err = wrap_to_pi(desired_heading - pose(3));
        boundary_scale = max(0.08, min(1.0, (boundary_d - boundary_stop) / max(boundary_warn - boundary_stop, 1e-6)));
        if boundary_d <= boundary_stop
            v_safe = min(v_safe, 0.05);
        else
            v_safe = min(v_safe, max(0.08, abs(v_safe) * boundary_scale));
        end
        w_safe = w_safe + 0.65 * boundary_heading_err;
    end
end

if d_eff <= stop_clear
    if trusted_track
        v_safe = min(v_safe, 0.10);
    else
        v_safe = min(v_safe, 0.04);
    end
elseif d_eff < warn_clear
    scale = max(0.12, min(1.0, (d_eff - stop_clear) / max(warn_clear - stop_clear, 1e-6)));
    if trusted_track
        v_safe = min(v_safe, max(0.12, abs(v_safe) * max(scale, 0.55)));
    else
        v_safe = min(v_safe, max(0.05, abs(v_safe) * scale));
    end
end

if d_eff < warn_clear && all(isfinite(closest_pt))
    heading_left = [-sin(pose(3)), cos(pose(3))];
    rel = pose(1:2) - closest_pt;
    side_val = dot(rel, heading_left);
    avoid_mag = 0.9 * max(0, warn_clear - d_eff) / max(warn_clear - stop_clear, 1e-6);
    if trusted_track
        avoid_mag = 0.30 * avoid_mag;
    end
    if side_val >= 0
        w_safe = w_safe + avoid_mag;
    else
        w_safe = w_safe - avoid_mag;
    end
end
end

function [v_safe, w_safe] = apply_path_distance_safety(v, w, public_vars)
v_safe = v;
w_safe = w;

if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end

q = public_vars.localization_quality;
if ~isfield(q, 'path_distance_m') || ~isfinite(q.path_distance_m)
    return;
end

path_dist = q.path_distance_m;
track_mode = isfield(public_vars, 'nav_state') && strcmp(public_vars.nav_state, 'track');

if track_mode
    if path_dist > 0.70
        v_safe = min(v_safe, 0.10);
    elseif path_dist > 0.50
        v_safe = min(v_safe, 0.16);
    elseif path_dist > 0.30
        v_safe = min(v_safe, 0.22);
    end
else
    if path_dist > 0.90
        v_safe = min(v_safe, 0.14);
    elseif path_dist > 0.65
        v_safe = min(v_safe, 0.20);
    elseif path_dist > 0.40
        v_safe = min(v_safe, 0.28);
    end
end

curve_mag = nan;
if isfield(public_vars, 'motion_debug') && isfield(public_vars.motion_debug, 'curvature') ...
        && isfinite(public_vars.motion_debug.curvature)
    curve_mag = abs(public_vars.motion_debug.curvature);
end
alpha_mag = nan;
if isfield(public_vars, 'motion_debug') && isfield(public_vars.motion_debug, 'alpha') ...
        && isfinite(public_vars.motion_debug.alpha)
    alpha_mag = abs(public_vars.motion_debug.alpha);
end

if track_mode && isfinite(curve_mag) && path_dist > 0.16
    if curve_mag > 0.60
        v_safe = min(v_safe, 0.20);
    elseif curve_mag > 0.40
        v_safe = min(v_safe, 0.26);
    elseif curve_mag > 0.28
        v_safe = min(v_safe, 0.30);
    end
end

if track_mode && isfinite(alpha_mag) && path_dist > 0.16
    if alpha_mag > 0.70
        v_safe = min(v_safe, 0.18);
    elseif alpha_mag > 0.45
        v_safe = min(v_safe, 0.24);
    elseif alpha_mag > 0.28
        v_safe = min(v_safe, 0.30);
    end
end
end

function [dmin, closest_pt] = min_wall_distance(p, walls)
dmin = inf;
closest_pt = [nan, nan];
for i = 1:size(walls, 1)
    [d, proj] = point_to_segment_distance_with_projection(p, walls(i, 1:2), walls(i, 3:4));
    if d < dmin
        dmin = d;
        closest_pt = proj;
    end
end
end

function [d, proj] = point_to_segment_distance_with_projection(p, a, b)
ab = b - a;
den = dot(ab, ab);
if den < 1e-12
    proj = a;
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / den;
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end

function [v_safe, w_safe] = apply_path_preview_safety(v, w, pose, path, state, read_only_vars, public_vars)
v_safe = v;
w_safe = w;

if ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars.map, 'walls') || isempty(read_only_vars.map.walls)
    return;
end
if size(path, 1) < 2 || numel(pose) < 3 || any(~isfinite(pose(1:3)))
    return;
end

[idx, ~] = nearest_path_idx(path, pose(1:2), state.path_idx);
preview_end = min(size(path, 1), idx + 2);
preview = [pose(1:2); path(idx:preview_end, 1:2)];
preview_clear = path_min_clearance_pm(preview, read_only_vars.map.walls, 0.02);

path_dist = inf;
if isfield(public_vars, 'localization_quality') && ~isempty(public_vars.localization_quality)
    q = public_vars.localization_quality;
    if isfield(q, 'path_distance_m') && isfinite(q.path_distance_m)
        path_dist = q.path_distance_m;
    end
end

tight_tracking = isfinite(path_dist) && path_dist <= 0.20;
if isfinite(path_dist) && path_dist > 1.0
    return;
end

warn_clear = 0.20;
stop_clear = 0.10;
if ~isfinite(preview_clear)
    return;
end
if preview_clear <= stop_clear
    if tight_tracking
        v_safe = min(v_safe, 0.22);
    else
        v_safe = min(v_safe, 0.10);
    end
elseif preview_clear < warn_clear
    if tight_tracking
        scale = max(0.72, min(1.0, (preview_clear - stop_clear) / max(warn_clear - stop_clear, 1e-6)));
        v_safe = min(v_safe, max(0.28, abs(v_safe) * scale));
    else
        scale = max(0.60, min(1.0, (preview_clear - stop_clear) / max(warn_clear - stop_clear, 1e-6)));
        v_safe = min(v_safe, max(0.22, abs(v_safe) * scale));
    end
end
end

function [v_safe, w_safe] = apply_goal_approach_safety(v, w, goal_dist)
v_safe = v;
w_safe = w;

if ~isfinite(goal_dist)
    return;
end

if goal_dist < 1.2
    v_safe = min(v_safe, max(0.08, 0.32 * goal_dist));
end
if goal_dist < 0.75
    v_safe = min(v_safe, 0.24);
    w_safe = max(min(w_safe, 0.45), -0.45);
end
if goal_dist < 0.45
    v_safe = min(v_safe, 0.12);
    w_safe = max(min(w_safe, 0.28), -0.28);
end
end

function dmin = path_min_clearance_pm(path, walls, ds)
dmin = inf;
for i = 1:(size(path, 1) - 1)
    pts = sample_segment_pm(path(i, :), path(i + 1, :), ds);
    for j = 1:size(pts, 1)
        for w = 1:size(walls, 1)
            [d, ~] = point_to_segment_distance_with_projection(pts(j, :), walls(w, 1:2), walls(w, 3:4));
            if d < dmin
                dmin = d;
            end
        end
    end
end
end

function pts = sample_segment_pm(a, b, ds)
L = norm(b - a);
if L < 1e-12
    pts = a;
    return;
end
n = max(2, ceil(L / max(ds, 1e-3)) + 1);
t = linspace(0, 1, n)';
pts = (1 - t) .* a + t .* b;
end

function value = getfield_with_default_pm(s, field_name, fallback)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = fallback;
end
end
