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
end

if ~isfield(public_vars, 'controller_mode') || isempty(public_vars.controller_mode)
    public_vars.controller_mode = 'pure_pursuit';
end

path = public_vars.path;
if isempty(path) || size(path, 1) < 2
    public_vars.motion_vector = [0, 0];
    return;
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

drive = read_only_vars.agent_drive;
Ts = read_only_vars.sampling_period;
goal = path(end, :);

if norm(pose(1:2) - goal) < 0.2
    public_vars.motion_vector = [0, 0];
    return;
end

% Pre-drive orientation stage:
% before moving, rotate in place if heading is strongly misaligned with the
% first path segment direction.
if isfield(public_vars, 'skip_start_alignment') && public_vars.skip_start_alignment
    state.start_heading_aligned = true;
end

if ~state.start_heading_aligned
    target_heading = initial_path_heading(path);
    e0 = wrap_to_pi(target_heading - pose(3));
    align_tol = 20 * pi / 180;     % [rad]
    release_tol = 12 * pi / 180;   % [rad], hysteresis
    k_align = 1.3;
    w_align_max = 0.40;
    w_align = max(min(k_align * e0, w_align_max), -w_align_max);

    if abs(e0) > align_tol
        public_vars.motion_vector = vw_to_wheels(0, w_align, drive);
        return;
    end
    if abs(e0) <= release_tol
        state.start_heading_aligned = true;
    end
end

switch lower(public_vars.controller_mode)
    case 'waypoint_p'
        [v, w, state] = ctrl_waypoint_p(pose, path, state);
    case 'pure_pursuit'
        [v, w, state] = ctrl_pure_pursuit(pose, path, state);
    case 'cross_track_pd'
        [v, w, state] = ctrl_cross_track_pd(pose, path, Ts, state);
    case 'stanley'
        [v, w, state] = ctrl_stanley(pose, path, state);
    otherwise
        [v, w, state] = ctrl_pure_pursuit(pose, path, state);
end

if ~isfinite(v) || ~isfinite(w)
    public_vars.motion_vector = [0, 0];
    return;
end

public_vars.motion_vector = vw_to_wheels(v, w, drive);

end

function pose = get_pose(read_only_vars, public_vars)
pose = [];

if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose)
    pose = public_vars.estimated_pose;
    return;
end

% No MoCap fallback by design.
end

function [v, w, state] = ctrl_waypoint_p(pose, path, state)
K_heading = 2.5;
v_nom = 0.55;
wp_tol = 0.25;

idx = state.wp_idx;
idx = min(max(idx, 1), size(path, 1));

while idx < size(path, 1) && norm(path(idx, :) - pose(1:2)) < wp_tol
    idx = idx + 1;
end
state.wp_idx = idx;

target = path(idx, :);
target_heading = atan2(target(2) - pose(2), target(1) - pose(1));
e_heading = wrap_to_pi(target_heading - pose(3));

v = v_nom * (1 - 0.5 * min(abs(e_heading) / pi, 1));
w = K_heading * e_heading;
end

function [v, w, state] = ctrl_pure_pursuit(pose, path, state)
v_nom = 0.6;
lookahead_dist = 0.7;
k_curve_slow = 0.9;

[nearest_idx, ~] = nearest_path_idx(path, pose(1:2), state.path_idx);
state.path_idx = nearest_idx;

look_idx = nearest_idx;
while look_idx < size(path, 1) && norm(path(look_idx, :) - pose(1:2)) < lookahead_dist
    look_idx = look_idx + 1;
end

target = path(look_idx, :);
alpha = wrap_to_pi(atan2(target(2) - pose(2), target(1) - pose(1)) - pose(3));
Ld = max(norm(target - pose(1:2)), 1e-3);
curvature = 2 * sin(alpha) / Ld;

v = v_nom / (1 + k_curve_slow * abs(curvature));
w = v * curvature;
end

function [v, w, state] = ctrl_cross_track_pd(pose, path, Ts, state)
K_cte = 0.65;
K_heading = 1.2;
K_d = 0.02;
v_nom = 0.5;
w_max = 1.4;

[seg_idx, e_ct, psi_path] = nearest_segment_error(path, pose(1:2), state.path_idx);
state.path_idx = seg_idx;

e_heading = wrap_to_pi(psi_path - pose(3));
de = (e_ct - state.prev_cte) / max(Ts, 1e-3);
state.prev_cte = e_ct;

w = K_heading * e_heading - K_cte * e_ct - K_d * de;
v = v_nom * (1 - 0.45 * min(abs(e_heading) / pi, 1));
w = max(min(w, w_max), -w_max);
end

function [v, w, state] = ctrl_stanley(pose, path, state)
k_stanley = 0.45;
K_w = 1.0;
v_nom = 0.45;
v_soft = 0.35;
w_max = 1.4;

[seg_idx, e_ct, psi_path] = nearest_segment_error(path, pose(1:2), state.path_idx);
state.path_idx = seg_idx;

e_heading = wrap_to_pi(psi_path - pose(3));
stanley_term = atan2(-k_stanley * e_ct, v_nom + v_soft);
w = K_w * (e_heading + stanley_term);
w = max(min(w, w_max), -w_max);

v = v_nom * (1 - 0.60 * min(abs(e_heading) / pi, 1));
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
