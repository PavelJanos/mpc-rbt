function [new_path] = smooth_path(old_path, read_only_vars, public_vars)
%SMOOTH_PATH Path smoothing with switchable algorithms.
% Modes:
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

mode = "iterative";
if isfield(public_vars, 'path_smoothing_mode') && ~isempty(public_vars.path_smoothing_mode)
    mode = lower(string(public_vars.path_smoothing_mode));
end

clearance_m = 0.2;
if isfield(public_vars, 'path_clearance_m') && isfinite(public_vars.path_clearance_m)
    clearance_m = max(0, public_vars.path_clearance_m);
end

walls = [];
if isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'walls')
    walls = read_only_vars.map.walls;
end

switch mode
    case "iterative"
        new_path = smooth_iterative(old_path, public_vars, walls, clearance_m);
        if ~isempty(walls)
            new_path = smooth_shortcut(new_path, walls, clearance_m);
        end
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

% Safety fallback: do not accept smoothed path if clearance is violated.
if ~isempty(walls)
    min_clear = path_min_clearance(new_path, walls, 0.03);
    if ~isfinite(min_clear) || min_clear < clearance_m
        new_path = old_path;
    end
end

end

function p = smooth_iterative(path, public_vars, walls, clearance_m)
alpha = 0.12;
beta = 0.48;
iter_max = 160;
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
iters = 2;
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
