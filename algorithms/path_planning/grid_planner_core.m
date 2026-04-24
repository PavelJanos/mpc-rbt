function [path, stats] = grid_planner_core(read_only_vars, public_vars, mode)
%GRID_PLANNER_CORE Shared occupancy-grid planner core.
% mode: 'astar' | 'dijkstra' | 'greedy'

path = [];
stats = struct('expanded', 0, 'success', false, 'cost', inf, 'mode', mode);

if ~isfield(read_only_vars, 'discrete_map') || isempty(read_only_vars.discrete_map)
    return;
end

dmap = read_only_vars.discrete_map;
grid = dmap.map;                  % 0 = free, 1 = occupied
limits = dmap.limits;             % [xmin ymin xmax ymax]
dims = dmap.dims;                 % [nx ny]
[ny, nx] = size(grid);            % map indexing: map(y, x)

if nx ~= dims(1) || ny ~= dims(2)
    dims = [nx, ny];
end

% Obstacle clearance handling (Task6/Task2): keep at least 0.2 m by default.
clearance_m = 0.2;
if isfield(public_vars, 'path_clearance_m') && isfinite(public_vars.path_clearance_m)
    clearance_m = max(0, public_vars.path_clearance_m);
end
grid = inflate_obstacles(grid, limits, dims, clearance_m);

start_xy = get_start_xy(read_only_vars, public_vars, limits);
goal_xy = get_goal_xy(read_only_vars, limits);

[sx, sy] = continous_to_discrete_coords(start_xy(1), start_xy(2), limits, dims);
[gx, gy] = continous_to_discrete_coords(goal_xy(1), goal_xy(2), limits, dims);

[sx, sy] = clip_to_grid(sx, sy, nx, ny);
[gx, gy] = clip_to_grid(gx, gy, nx, ny);

[sx, sy] = snap_to_nearest_free(grid, sx, sy);
[gx, gy] = snap_to_nearest_free(grid, gx, gy);

if grid(sy, sx) ~= 0 || grid(gy, gx) ~= 0
    return;
end

start_idx = sub2ind([ny, nx], sy, sx);
goal_idx = sub2ind([ny, nx], gy, gx);

if start_idx == goal_idx
    path = [start_xy; goal_xy];
    stats.success = true;
    stats.cost = 0;
    return;
end

g_score = inf(ny, nx);
parent = zeros(ny, nx, 'uint32');
closed = false(ny, nx);
open = false(ny, nx);
f_score = inf(ny, nx);

g_score(sy, sx) = 0;
f_score(sy, sx) = heuristic(sx, sy, gx, gy, mode);
open(sy, sx) = true;

dx = [-1, 0, 1, -1, 1, -1, 0, 1];
dy = [-1, -1, -1, 0, 0, 1, 1, 1];
step_cost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

while any(open(:))
    open_idx = find(open);
    [~, i_best] = min(f_score(open_idx));
    current_idx = open_idx(i_best);
    [cy, cx] = ind2sub([ny, nx], current_idx);

    open(cy, cx) = false;
    closed(cy, cx) = true;
    stats.expanded = stats.expanded + 1;

    if current_idx == goal_idx
        stats.success = true;
        stats.cost = g_score(cy, cx);
        break;
    end

    for k = 1:8
        nx_i = cx + dx(k);
        ny_i = cy + dy(k);
        if nx_i < 1 || nx_i > nx || ny_i < 1 || ny_i > ny
            continue;
        end
        if closed(ny_i, nx_i) || grid(ny_i, nx_i) ~= 0
            continue;
        end

        % No corner cutting through occupied cells.
        if dx(k) ~= 0 && dy(k) ~= 0
            if grid(cy, nx_i) ~= 0 || grid(ny_i, cx) ~= 0
                continue;
            end
        end

        cand_g = g_score(cy, cx) + step_cost(k);
        n_h = heuristic(nx_i, ny_i, gx, gy, mode);

        switch mode
            case 'greedy'
                better = ~open(ny_i, nx_i) || (n_h < f_score(ny_i, nx_i));
                cand_f = n_h;
            case 'dijkstra'
                better = cand_g < g_score(ny_i, nx_i);
                cand_f = cand_g;
            otherwise % astar
                better = cand_g < g_score(ny_i, nx_i);
                cand_f = cand_g + n_h;
        end

        if better
            parent(ny_i, nx_i) = uint32(current_idx);
            g_score(ny_i, nx_i) = cand_g;
            f_score(ny_i, nx_i) = cand_f;
            open(ny_i, nx_i) = true;
        end
    end
end

if ~stats.success
    return;
end

% Reconstruct path in grid coordinates.
idx = goal_idx;
xy_idx = zeros(0, 2);
while idx ~= 0
    [py, px] = ind2sub([ny, nx], idx);
    xy_idx(end + 1, :) = [px, py]; %#ok<AGROW>
    idx = parent(py, px);
end
xy_idx = flipud(xy_idx);

% Convert grid -> world coordinates.
path = zeros(size(xy_idx, 1), 2);
for i = 1:size(xy_idx, 1)
    path(i, :) = discrete_to_world(xy_idx(i, 1), xy_idx(i, 2), limits, dims);
end

% Anchor endpoints to exact continuous start/goal.
path(1, :) = start_xy;
path(end, :) = goal_xy;

end

function h = heuristic(x, y, gx, gy, mode)
switch mode
    case 'dijkstra'
        h = 0;
    otherwise
        h = hypot(double(gx - x), double(gy - y));
end
end

function [ix, iy] = clip_to_grid(ix, iy, nx, ny)
ix = max(1, min(nx, ix));
iy = max(1, min(ny, iy));
end

function [ix, iy] = snap_to_nearest_free(grid, ix, iy)
[ny, nx] = size(grid);
[ix, iy] = clip_to_grid(ix, iy, nx, ny);

if grid(iy, ix) == 0
    return;
end

max_r = max(nx, ny);
for r = 1:max_r
    x_min = max(1, ix - r);
    x_max = min(nx, ix + r);
    y_min = max(1, iy - r);
    y_max = min(ny, iy + r);
    for y = y_min:y_max
        for x = x_min:x_max
            if grid(y, x) == 0
                ix = x;
                iy = y;
                return;
            end
        end
    end
end
end

function grid_out = inflate_obstacles(grid_in, limits, dims, clearance_m)
grid_out = grid_in;
if clearance_m <= 0
    return;
end

nx = dims(1);
ny = dims(2);
if nx <= 1 || ny <= 1
    return;
end

dx = (limits(3) - limits(1)) / (nx - 1);
dy = (limits(4) - limits(2)) / (ny - 1);
cell_m = max(dx, dy);
if ~isfinite(cell_m) || cell_m <= 0
    return;
end

r = ceil(clearance_m / cell_m);
if r <= 0
    return;
end

[yy, xx] = ndgrid(-r:r, -r:r);
se = (xx.^2 + yy.^2) <= r^2;

occ = grid_in ~= 0;
inflated = conv2(double(occ), double(se), 'same') > 0;
grid_out(inflated) = 1;
end

function xy = discrete_to_world(ix, iy, limits, dims)
nx = dims(1);
ny = dims(2);

if nx <= 1
    x = limits(1);
else
    x = limits(1) + (ix - 1) * (limits(3) - limits(1)) / (nx - 1);
end

if ny <= 1
    y = limits(2);
else
    y = limits(2) + (iy - 1) * (limits(4) - limits(2)) / (ny - 1);
end

xy = [x, y];
end

function xy = get_start_xy(read_only_vars, public_vars, limits)
if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose) ...
        && all(isfinite(public_vars.estimated_pose(1:2)))
    xy = public_vars.estimated_pose(1:2);
    return;
end

if isfield(read_only_vars, 'gnss_position') && ~isempty(read_only_vars.gnss_position) ...
        && all(isfinite(read_only_vars.gnss_position(1:2)))
    xy = read_only_vars.gnss_position(1:2);
    return;
end

xy = [limits(1), limits(2)];
end

function xy = get_goal_xy(read_only_vars, limits)
if isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'goal') ...
        && ~isempty(read_only_vars.map.goal) && all(isfinite(read_only_vars.map.goal(1:2)))
    xy = read_only_vars.map.goal(1:2);
    return;
end

if isfield(read_only_vars, 'discrete_map') && isfield(read_only_vars.discrete_map, 'goal') ...
        && ~isempty(read_only_vars.discrete_map.goal)
    g = read_only_vars.discrete_map.goal;
    if numel(g) >= 2
        % Convert discrete goal to world if continuous goal is unavailable.
        xy = [limits(1) + g(1) - 1, limits(2) + g(2) - 1];
        return;
    end
end

xy = [limits(3), limits(4)];
end
