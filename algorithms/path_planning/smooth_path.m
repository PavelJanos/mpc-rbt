function [new_path] = smooth_path(old_path, read_only_vars, public_vars)
%SMOOTH_PATH Path smoothing with switchable algorithms.
% Modes:
% - 'none'      : return raw planner path
% - 'iterative' : gradient-like iterative smoothing (lecture style)
% - 'shortcut'  : line-of-sight waypoint reduction
% - 'spline'    : spline interpolation on arc-length
% - 'chaikin'   : corner-cutting subdivision

if nargin < 2
    read_only_vars = struct();
end
if nargin < 3
    public_vars = struct();
end

if isempty(old_path) || size(old_path, 1) <= 2
    new_path = old_path;
    return;
end

mode = "chaikin";
if isfield(public_vars, 'path_smoothing_mode') && ~isempty(public_vars.path_smoothing_mode)
    mode = lower(string(public_vars.path_smoothing_mode));
end

clearance_m = 0.25;
if isfield(public_vars, 'tracking_clearance_m') && isfinite(public_vars.tracking_clearance_m)
    clearance_m = max(0, public_vars.tracking_clearance_m);
elseif isfield(public_vars, 'path_clearance_m') && isfinite(public_vars.path_clearance_m)
    clearance_m = max(0, public_vars.path_clearance_m);
end

resample_ds = 0.10;
if isfield(public_vars, 'path_resample_ds_m') && isfinite(public_vars.path_resample_ds_m)
    resample_ds = max(0.05, public_vars.path_resample_ds_m);
end

refine_iters = 1;
if isfield(public_vars, 'path_refine_iters') && isfinite(public_vars.path_refine_iters)
    refine_iters = max(0, round(public_vars.path_refine_iters));
end

walls = [];
if isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'walls')
    walls = read_only_vars.map.walls;
end

switch mode
    case "none"
        new_path = old_path;
    case "iterative"
        new_path = smooth_iterative(old_path, public_vars, walls, clearance_m);
        if ~isempty(walls)
            new_path = smooth_shortcut(new_path, walls, clearance_m);
        end
        new_path = refine_path(new_path, old_path, walls, clearance_m, refine_iters);
    case "shortcut"
        new_path = smooth_shortcut(old_path, walls, clearance_m);
    case "spline"
        new_path = smooth_spline(old_path, public_vars);
    case "chaikin"
        new_path = smooth_chaikin(old_path, public_vars);
    otherwise
        new_path = smooth_iterative(old_path, public_vars, walls, clearance_m);
end

% Keep fixed endpoints.
new_path(1, :) = old_path(1, :);
new_path(end, :) = old_path(end, :);

raw_len = path_length(old_path);
new_len = path_length(new_path);

if ~isfinite(new_len) || new_len > 1.10 * max(raw_len, 1e-6) || path_self_intersects(new_path)
    fallback_path = old_path;
    if ~isempty(walls)
        shortcut_path = smooth_shortcut(old_path, walls, clearance_m);
        shortcut_len = path_length(shortcut_path);
        if ~path_self_intersects(shortcut_path) && isfinite(shortcut_len) ...
                && shortcut_len <= 1.05 * max(raw_len, 1e-6)
            fallback_path = shortcut_path;
        end
    end
    new_path = fallback_path;
end

% Safety fallback: do not accept smoothed path if clearance is violated.
if ~isempty(walls)
    min_clear = path_min_clearance(new_path, walls, 0.03);
    if ~isfinite(min_clear) || min_clear < clearance_m
        new_path = old_path;
    end
end

new_path = resample_path(new_path, resample_ds);

end

function p = smooth_iterative(path, public_vars, walls, clearance_m)
alpha = 0.10;
beta = 0.58;
iter_max = 220;
tol = 1e-4;

if isfield(public_vars, 'smooth_iter_alpha') && isfinite(public_vars.smooth_iter_alpha)
    alpha = public_vars.smooth_iter_alpha;
end
if isfield(public_vars, 'smooth_iter_beta') && isfinite(public_vars.smooth_iter_beta)
    beta = public_vars.smooth_iter_beta;
end
if isfield(public_vars, 'smooth_iter_max') && isfinite(public_vars.smooth_iter_max)
    iter_max = max(1, round(public_vars.smooth_iter_max));
end
if isfield(public_vars, 'smooth_iter_tol') && isfinite(public_vars.smooth_iter_tol)
    tol = max(1e-8, public_vars.smooth_iter_tol);
end

ref = path;
p = path;
n = size(path, 1);

for it = 1:iter_max
    p_prev = p;
    for i = 2:(n - 1)
        cand = p(i, :) ...
            + alpha * (ref(i, :) - p(i, :)) ...
            + beta * (p(i - 1, :) + p(i + 1, :) - 2 * p(i, :));

        if isempty(walls)
            p(i, :) = cand;
            continue;
        end

        % Apply only local updates that keep required clearance.
        if point_clear(cand, walls, clearance_m) ...
                && segment_clear(p(i - 1, :), cand, walls, clearance_m) ...
                && segment_clear(cand, p(i + 1, :), walls, clearance_m)
            p(i, :) = cand;
        end
    end
    p(1, :) = ref(1, :);
    p(end, :) = ref(end, :);

    if max(vecnorm(p - p_prev, 2, 2)) < tol
        break;
    end
end

function tf = point_clear(p, walls, clearance_m)
for w = 1:size(walls, 1)
    d = point_to_segment_distance(p, walls(w, 1:2), walls(w, 3:4));
    if d < clearance_m
        tf = false;
        return;
    end
end
tf = true;
end
end

function p = refine_path(path, ref_path, walls, clearance_m, refine_iters)
p = path;
if refine_iters <= 0 || size(path, 1) <= 2
    return;
end

for t = 1:refine_iters
    cand = smooth_chaikin(p, struct('smooth_chaikin_iters', 1));
    cand(1, :) = ref_path(1, :);
    cand(end, :) = ref_path(end, :);
    if ~isempty(walls)
        min_clear = path_min_clearance(cand, walls, 0.03);
        if ~isfinite(min_clear) || min_clear < clearance_m || path_self_intersects(cand)
            break;
        end
    end
    p = cand;
end
end

function p = smooth_shortcut(path, walls, clearance_m)
if isempty(walls)
    p = path;
    return;
end

p = path(1, :);
i = 1;
n = size(path, 1);

while i < n
    best_j = i + 1;
    j = i + 1;
    while j <= n
        if segment_clear(path(i, :), path(j, :), walls, clearance_m)
            best_j = j;
            j = j + 1;
        else
            break;
        end
    end
    p(end + 1, :) = path(best_j, :); %#ok<AGROW>
    i = best_j;
end
end

function p = smooth_spline(path, public_vars)
gain = 3;
if isfield(public_vars, 'smooth_spline_gain') && isfinite(public_vars.smooth_spline_gain)
    gain = max(2, public_vars.smooth_spline_gain);
end

s = [0; cumsum(vecnorm(diff(path, 1, 1), 2, 2))];
if s(end) <= 1e-9
    p = path;
    return;
end

nq = max(3 * size(path, 1), round(gain * size(path, 1)));
sq = linspace(0, s(end), nq)';
xq = spline(s, path(:, 1), sq);
yq = spline(s, path(:, 2), sq);
p = [xq, yq];
end

function p = smooth_chaikin(path, public_vars)
iters = 4;
if isfield(public_vars, 'smooth_chaikin_iters') && isfinite(public_vars.smooth_chaikin_iters)
    iters = max(1, round(public_vars.smooth_chaikin_iters));
end

p = path;
for t = 1:iters
    q = zeros(2 * size(p, 1) - 2, 2);
    idx = 1;
    for i = 1:(size(p, 1) - 1)
        a = p(i, :);
        b = p(i + 1, :);
        q(idx, :) = 0.75 * a + 0.25 * b; idx = idx + 1;
        q(idx, :) = 0.25 * a + 0.75 * b; idx = idx + 1;
    end
    p = [p(1, :); q; p(end, :)];
end
end

function tf = segment_clear(a, b, walls, clearance_m)
pts = sample_segment(a, b, 0.03);
for i = 1:size(pts, 1)
    pi = pts(i, :);
    for w = 1:size(walls, 1)
        d = point_to_segment_distance(pi, walls(w, 1:2), walls(w, 3:4));
        if d < clearance_m
            tf = false;
            return;
        end
    end
end
tf = true;
end

function dmin = path_min_clearance(path, walls, ds)
if nargin < 3
    ds = 0.03;
end
pts = sample_path(path, ds);
dmin = inf;
for i = 1:size(pts, 1)
    pi = pts(i, :);
    for w = 1:size(walls, 1)
        d = point_to_segment_distance(pi, walls(w, 1:2), walls(w, 3:4));
        if d < dmin
            dmin = d;
        end
    end
end
end

function L = path_length(path)
if isempty(path) || size(path, 1) < 2
    L = 0;
    return;
end
L = sum(vecnorm(diff(path, 1, 1), 2, 2));
end

function tf = path_self_intersects(path)
tf = false;
n = size(path, 1);
if n < 4
    return;
end

for i = 1:(n - 1)
    a1 = path(i, :);
    a2 = path(i + 1, :);
    for j = (i + 2):(n - 1)
        if j == i + 1
            continue;
        end
        if i == 1 && j == n - 1
            continue;
        end
        b1 = path(j, :);
        b2 = path(j + 1, :);
        if segments_intersect(a1, a2, b1, b2)
            tf = true;
            return;
        end
    end
end
end

function tf = segments_intersect(a1, a2, b1, b2)
eps_orient = 1e-9;
o1 = orientation2d(a1, a2, b1);
o2 = orientation2d(a1, a2, b2);
o3 = orientation2d(b1, b2, a1);
o4 = orientation2d(b1, b2, a2);

tf = false;
if o1 * o2 < -eps_orient && o3 * o4 < -eps_orient
    tf = true;
    return;
end

if abs(o1) <= eps_orient && on_segment(a1, a2, b1)
    tf = true;
elseif abs(o2) <= eps_orient && on_segment(a1, a2, b2)
    tf = true;
elseif abs(o3) <= eps_orient && on_segment(b1, b2, a1)
    tf = true;
elseif abs(o4) <= eps_orient && on_segment(b1, b2, a2)
    tf = true;
end
end

function o = orientation2d(a, b, c)
ab = b - a;
ac = c - a;
o = ab(1) * ac(2) - ab(2) * ac(1);
end

function tf = on_segment(a, b, p)
tol = 1e-9;
tf = p(1) >= min(a(1), b(1)) - tol && p(1) <= max(a(1), b(1)) + tol ...
    && p(2) >= min(a(2), b(2)) - tol && p(2) <= max(a(2), b(2)) + tol;
end

function p = resample_path(path, ds)
if isempty(path) || size(path, 1) <= 2
    p = path;
    return;
end

p = path(1, :);
for i = 1:(size(path, 1) - 1)
    seg = sample_segment(path(i, :), path(i + 1, :), ds);
    if i > 1
        seg(1, :) = [];
    end
    p = [p; seg]; %#ok<AGROW>
end
end

function pts = sample_segment(a, b, ds)
L = norm(b - a);
if L < 1e-12
    pts = a;
    return;
end
n = max(2, ceil(L / max(ds, 1e-3)) + 1);
t = linspace(0, 1, n)';
pts = (1 - t) .* a + t .* b;
end

function pts = sample_path(path, ds)
pts = path(1, :);
for i = 1:(size(path, 1) - 1)
    seg = sample_segment(path(i, :), path(i + 1, :), ds);
    if i > 1
        seg(1, :) = [];
    end
    pts = [pts; seg]; %#ok<AGROW>
end
end

function d = point_to_segment_distance(p, a, b)
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
