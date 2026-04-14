function [path, raw_path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

% Reuse current path unless explicit replanning is requested.
if isfield(public_vars, 'path') && ~isempty(public_vars.path) ...
        && ~(isfield(public_vars, 'replan_path') && public_vars.replan_path)
    path = public_vars.path;
    raw_path = path;
    return;
end

planner_mode = 'astar';
if isfield(public_vars, 'path_planner_mode') && ~isempty(public_vars.path_planner_mode)
    planner_mode = lower(string(public_vars.path_planner_mode));
end

planner_public = public_vars;
planner_public.path_clearance_m = get_planner_clearance_or(public_vars, 0.25);
raw_path = run_planner(planner_mode, read_only_vars, planner_public);

if isempty(raw_path) || size(raw_path, 1) < 2
    fallback_clearances = [0.20, 0.16, 0.12];
    base_clearance = get_planner_clearance_or(public_vars, 0.25);
    for i = 1:numel(fallback_clearances)
        c_try = fallback_clearances(i);
        if c_try >= base_clearance - 1e-6
            continue;
        end
        tmp_public = public_vars;
        tmp_public.path_clearance_m = c_try;
        raw_path = run_planner(planner_mode, read_only_vars, tmp_public);
        if ~isempty(raw_path) && size(raw_path, 1) >= 2
            break;
        end
    end
end

smooth_public = public_vars;
smooth_public.path_clearance_m = get_tracking_clearance_or(public_vars, get_planner_clearance_or(public_vars, 0.25));
path = choose_best_smoothed_path(raw_path, read_only_vars, smooth_public);

end

function raw_path = run_planner(planner_mode, read_only_vars, public_vars)
switch planner_mode
    case "astar"
        raw_path = astar(read_only_vars, public_vars);
    case "dijkstra"
        raw_path = dijkstra(read_only_vars, public_vars);
    case "greedy"
        raw_path = greedy_best_first(read_only_vars, public_vars);
    otherwise
        raw_path = astar(read_only_vars, public_vars);
end
end

function value = get_planner_clearance_or(public_vars, fallback)
if isfield(public_vars, 'planner_clearance_m') && isfinite(public_vars.planner_clearance_m)
    value = public_vars.planner_clearance_m;
elseif isfield(public_vars, 'path_clearance_m') && isfinite(public_vars.path_clearance_m)
    value = public_vars.path_clearance_m;
else
    value = fallback;
end
end

function value = get_tracking_clearance_or(public_vars, fallback)
if isfield(public_vars, 'tracking_clearance_m') && isfinite(public_vars.tracking_clearance_m)
    value = public_vars.tracking_clearance_m;
elseif isfield(public_vars, 'path_clearance_m') && isfinite(public_vars.path_clearance_m)
    value = public_vars.path_clearance_m;
else
    value = fallback;
end
end

function best_path = choose_best_smoothed_path(raw_path, read_only_vars, public_vars)
best_path = smooth_path(raw_path, read_only_vars, public_vars);
best_score = local_path_quality_score(best_path, raw_path, read_only_vars, public_vars);
if ~isfinite(best_score)
    best_score = -inf;
end

if best_score >= 0.60 || isempty(raw_path) || size(raw_path, 1) < 3
    return;
end

candidates = {};
candidates{end + 1} = raw_path; %#ok<AGROW>
if isfield(public_vars, 'path_smoothing_mode') && ~strcmpi(string(public_vars.path_smoothing_mode), "chaikin")
    tmp = public_vars;
    tmp.path_smoothing_mode = 'chaikin';
    candidates{end + 1} = smooth_path(raw_path, read_only_vars, tmp); %#ok<AGROW>
end
tmp = public_vars;
tmp.smooth_chaikin_iters = getfield_or(tmp, 'smooth_chaikin_iters', 3) + 1;
tmp.path_smoothing_mode = 'chaikin';
candidates{end + 1} = smooth_path(raw_path, read_only_vars, tmp); %#ok<AGROW>
tmp = public_vars;
tmp.path_smoothing_mode = 'iterative';
candidates{end + 1} = smooth_path(raw_path, read_only_vars, tmp); %#ok<AGROW>
tmp = public_vars;
tmp.path_smoothing_mode = 'shortcut';
candidates{end + 1} = smooth_path(raw_path, read_only_vars, tmp); %#ok<AGROW>

for i = 1:numel(candidates)
    cand = candidates{i};
    cand_score = local_path_quality_score(cand, raw_path, read_only_vars, public_vars);
    if cand_score > best_score + 1e-6
        best_path = cand;
        best_score = cand_score;
    end
end
end

function score = local_path_quality_score(path, raw_path, read_only_vars, public_vars)
score = -inf;
if isempty(path) || size(path, 1) < 2
    return;
end

walls = [];
if isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'walls')
    walls = read_only_vars.map.walls;
end
clearance_target = get_tracking_clearance_or(public_vars, 0.25);
min_clear = inf;
if ~isempty(walls)
    min_clear = path_min_clearance_local(path, walls, 0.03);
    if ~isfinite(min_clear) || min_clear < clearance_target - 1e-6
        return;
    end
end

seg = diff(path(:, 1:2), 1, 1);
seg_len = vecnorm(seg, 2, 2);
angles = atan2(seg(:, 2), seg(:, 1));
max_turn = 0;
turn_density = 0;
if numel(angles) >= 2
    dtheta = abs(mod(diff(angles) + pi, 2 * pi) - pi);
    max_turn = max(dtheta) * 180 / pi;
    turn_density = sum(dtheta) / max(sum(seg_len), 1e-6);
end
raw_len = sum(vecnorm(diff(raw_path(:, 1:2), 1, 1), 2, 2));
path_len = sum(seg_len);
length_ratio = path_len / max(raw_len, 1e-6);

score = 1.0 ...
    - 0.48 * max(0, 1 - min_clear / max(clearance_target, 1e-6)) ...
    - 0.28 * max(0, (max_turn - 42) / 70) ...
    - 0.16 * max(0, (turn_density - 0.42) / 0.70) ...
    - 0.16 * max(0, length_ratio - 1.08);
end

function value = getfield_or(s, field_name, fallback)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = fallback;
end
end

function dmin = path_min_clearance_local(path, walls, ds)
if nargin < 3
    ds = 0.03;
end
pts = sample_path_local(path, ds);
dmin = inf;
for i = 1:size(pts, 1)
    pi = pts(i, :);
    for w = 1:size(walls, 1)
        d = point_to_segment_distance_local(pi, walls(w, 1:2), walls(w, 3:4));
        if d < dmin
            dmin = d;
        end
    end
end
end

function pts = sample_path_local(path, ds)
pts = path(1, :);
for i = 1:(size(path, 1) - 1)
    seg = sample_segment_local(path(i, :), path(i + 1, :), ds);
    if i > 1
        seg(1, :) = [];
    end
    pts = [pts; seg]; %#ok<AGROW>
end
end

function pts = sample_segment_local(a, b, ds)
L = norm(b - a);
if L < 1e-12
    pts = a;
    return;
end
n = max(2, ceil(L / max(ds, 1e-3)) + 1);
t = linspace(0, 1, n)';
pts = (1 - t) .* a + t .* b;
end

function d = point_to_segment_distance_local(p, a, b)
ab = b - a;
den = dot(ab, ab);
if den < 1e-12
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / den;
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end

