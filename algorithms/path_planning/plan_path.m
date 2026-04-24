function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

% Reuse current path unless explicit replanning is requested.
if isfield(public_vars, 'path') && ~isempty(public_vars.path) ...
        && ~(isfield(public_vars, 'replan_path') && public_vars.replan_path)
    path = public_vars.path;
    return;
end

force_grid = isfield(public_vars, 'force_grid_planner') && public_vars.force_grid_planner;

if ~force_grid && is_indoor_1_map(read_only_vars.map)
    % Task3/Task2: handcrafted safe path with curved segments.
    path = create_task3_task2_path();
    return;
end

if ~force_grid && is_outdoor_1_map(read_only_vars.map)
    % Task5/Task1: handcrafted trajectory from [2,2] to [16,2].
    path = create_task5_task1_path();
    return;
end

planner_mode = 'astar';
if isfield(public_vars, 'path_planner_mode') && ~isempty(public_vars.path_planner_mode)
    planner_mode = lower(string(public_vars.path_planner_mode));
end

switch planner_mode
    case "astar"
        path = astar(read_only_vars, public_vars);
    case "dijkstra"
        path = dijkstra(read_only_vars, public_vars);
    case "greedy"
        path = greedy_best_first(read_only_vars, public_vars);
    otherwise
        path = astar(read_only_vars, public_vars);
end

path = smooth_path(path, read_only_vars, public_vars);

end

function tf = is_indoor_1_map(map)
tf = isequal(size(map.walls), [6, 4]) ...
    && isequal(round(map.goal, 6), [9, 9]) ...
    && isequal(round(map.limits, 6), [0, 0, 10, 10]);
end

function tf = is_outdoor_1_map(map)
tf = isequal(size(map.walls), [15, 4]) ...
    && isequal(round(map.goal, 6), [16, 2]) ...
    && isequal(round(map.limits, 6), [0, 0, 20, 15]);
end

function path = create_task3_task2_path()
% Start point required by assignment
% From (2, 8.5) to goal (9, 9), includes curved segments.

path = [];

% 1) Curved segment (Bezier) above the first vertical wall (x=3.3, y<=7)
t = linspace(0, 1, 50)';
P0 = [2.0, 8.5];
P1 = [2.8, 9.2];
P2 = [3.6, 8.8];
P3 = [4.2, 7.9];
bezier = (1 - t).^3 .* P0 + ...
    3 * (1 - t).^2 .* t .* P1 + ...
    3 * (1 - t) .* t.^2 .* P2 + ...
    t.^3 .* P3;
path = [path; bezier];

% 2) Straight descent on the right side of the first wall
y2 = linspace(7.9, 2.2, 55)';
path = [path; [4.2 * ones(numel(y2), 1), y2]];

% 3) Curved low corridor below the second wall endpoint (x=7.2, y=3)
x3 = linspace(4.2, 8.6, 80)';
y3 = 2.2 + 0.3 * sin(2 * pi * (x3 - 4.2) / (8.6 - 4.2));
path = [path; [x3, y3]];

% 4) Circular arc for smooth heading change before final climb
theta = linspace(-pi/2, 0, 24)';
x4 = 8.6 + 0.8 * cos(theta);
y4 = 3.0 + 0.8 * sin(theta);
path = [path; [x4, y4]];

% 5) Final ascent to goal
x5 = linspace(9.4, 9.0, 70)';
y5 = linspace(3.0, 9.0, 70)';
path = [path; [x5, y5]];

% Ensure exact goal as the last waypoint.
path(end, :) = [9, 9];
end

function path = create_task5_task1_path()
% Task5/Task1: manual trajectory for outdoor_1
% Start [2,2] -> Goal [16,2], includes curved segments.

path = [];

% 1) Vertical segment up from start.
y1 = linspace(2, 8, 55)';
path = [path; [2 * ones(numel(y1), 1), y1]];

% 2) Curved horizontal traverse in upper corridor.
x2 = linspace(2, 11, 90)';
y2 = 8 + 0.6 * sin(2 * pi * (x2 - 2) / (11 - 2));
path = [path; [x2, y2]];

% 3) Smooth bezier descent to goal.
t = linspace(0, 1, 85)';
P0 = [11, 8];
P1 = [13, 8.2];
P2 = [15, 4.4];
P3 = [16, 2];
bezier = (1 - t).^3 .* P0 + ...
    3 * (1 - t).^2 .* t .* P1 + ...
    3 * (1 - t) .* t.^2 .* P2 + ...
    t.^3 .* P3;
path = [path; bezier];

path(1, :) = [2, 2];
path(end, :) = [16, 2];
end

