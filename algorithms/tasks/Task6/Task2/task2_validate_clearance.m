%% Task6/Task2 - Obstacle clearance validation (>= 0.2 m)
clear; clc; close all;

project_root = fileparts(mfilename('fullpath'));
while ~exist(fullfile(project_root, 'main.m'), 'file')
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        error('Could not locate project root (main.m).');
    end
    project_root = parent;
end

addpath(fullfile(project_root, 'utils'));
addpath(fullfile(project_root, 'algorithms'));
addpath(fullfile(project_root, 'algorithms', 'path_planning'));

out_dir = fileparts(mfilename('fullpath'));

map = load_map(fullfile(project_root, 'maps', 'indoor_1.txt'));
map.discretization_step = 0.2;
dmap = generate_discrete_map(map);

start_pose = [2.0, 1.0, -pi/2];
required_clearance = 0.2;
planners = {'astar', 'dijkstra', 'greedy'};

read_only = struct();
read_only.map = map;
read_only.discrete_map = dmap;
read_only.mocap_pose = start_pose;
read_only.counter = 1;

rows = cell(numel(planners), 1);
paths = cell(numel(planners), 1);

for i = 1:numel(planners)
    public_vars = struct();
    public_vars.path = [];
    public_vars.replan_path = true;
    public_vars.force_grid_planner = true;
    public_vars.path_planner_mode = planners{i};
    public_vars.path_clearance_m = required_clearance;

    p = plan_path(read_only, public_vars);
    paths{i} = p;
    ok = ~isempty(p) && size(p, 1) >= 2;
    if ok
        min_clear = path_min_clearance(p, map.walls);
        plen = sum(vecnorm(diff(p,1,1), 2, 2));
    else
        min_clear = 0;
        plen = inf;
    end
    clearance_ok = ok && (min_clear >= required_clearance);
    rows{i} = {planners{i}, ok, clearance_ok, min_clear, plen, size(p,1)};
end

tbl = cell2table(vertcat(rows{:}), 'VariableNames', ...
    {'planner','path_found','clearance_ok','min_clearance_m','path_length_m','waypoints'});
writetable(tbl, fullfile(out_dir, 'task2_clearance_table.csv'));

% Figure
fig = figure('Color', 'w', 'Position', [120 120 1200 700], 'Name', 'Task6/Task2 clearance');
hold on; axis equal; grid on;
for i = 1:size(map.walls,1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 4, 'HandleVisibility', 'off');
end

styles = {'-','--',':'};
colors = [0.10 0.45 0.95; 0.85 0.33 0.10; 0.20 0.70 0.25];
for i = 1:numel(planners)
    p = paths{i};
    if ~isempty(p)
        label = sprintf('%s (min=%.3f m)', planners{i}, tbl.min_clearance_m(i));
        plot(p(:,1), p(:,2), styles{i}, 'Color', colors(i,:), 'LineWidth', 2.5, 'DisplayName', label);
    end
end
plot(start_pose(1), start_pose(2), 'kp', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Start');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 11, 'LineWidth', 2, 'DisplayName', 'Goal');
title(sprintf('Task6/Task2: Clearance validation (required >= %.2f m)', required_clearance), 'FontWeight', 'bold');
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5, map.limits(3)+0.5, map.limits(2)-0.5, map.limits(4)+0.5]);

png_path = fullfile(out_dir, 'task2_clearance_validation.png');
fig_path = fullfile(out_dir, 'task2_clearance_validation.fig');
savefig(fig, fig_path);
exportgraphics(fig, png_path, 'BackgroundColor', 'white', 'Resolution', 170);

% Report
rep = fopen(fullfile(out_dir, 'task2_clearance_report.md'), 'w');
fprintf(rep, '# Task6/Task2 - Odstup od prekazek\n\n');
fprintf(rep, '- Pozadovany minimalni odstup: `%.2f m`\n', required_clearance);
fprintf(rep, '- Metoda: inflace prekazek v occupancy mape o %.2f m v planneru.\n\n', required_clearance);
fprintf(rep, '## Vysledky\n\n');
fprintf(rep, '| Planner | Trasa nalezena | Odstup splnen | Min odstup [m] | Delka [m] | Waypointy |\n');
fprintf(rep, '|---|---:|---:|---:|---:|---:|\n');
for i = 1:height(tbl)
    fprintf(rep, '| %s | %d | %d | %.3f | %.3f | %d |\n', ...
        tbl.planner{i}, tbl.path_found(i), tbl.clearance_ok(i), ...
        tbl.min_clearance_m(i), tbl.path_length_m(i), tbl.waypoints(i));
end
fprintf(rep, '\n## Vystupy\n');
fprintf(rep, '- `task2_clearance_validation.png`\n');
fprintf(rep, '- `task2_clearance_table.csv`\n');
fprintf(rep, '- `task2_clearance_report.md`\n');
fclose(rep);

fprintf('Saved: %s\n', rel_path(png_path, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task2_clearance_table.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task2_clearance_report.md'), project_root));

function dmin = path_min_clearance(path, walls)
samples = sample_path(path, 0.03);
dmin = inf;
for i = 1:size(samples,1)
    p = samples(i,:);
    for w = 1:size(walls,1)
        a = walls(w,1:2);
        b = walls(w,3:4);
        d = point_to_segment_distance(p, a, b);
        if d < dmin
            dmin = d;
        end
    end
end
end

function pts = sample_path(path, ds)
pts = path(1,:);
for i = 1:(size(path,1)-1)
    a = path(i,:);
    b = path(i+1,:);
    L = norm(b-a);
    if L < 1e-12
        continue;
    end
    n = max(2, ceil(L/max(ds,1e-3))+1);
    t = linspace(0,1,n)';
    seg = (1-t).*a + t.*b;
    if i > 1
        seg(1,:) = [];
    end
    pts = [pts; seg]; %#ok<AGROW>
end
end

function d = point_to_segment_distance(p, a, b)
ab = b - a;
if dot(ab,ab) < 1e-12
    d = norm(p-a);
    return;
end
t = dot(p-a, ab) / dot(ab,ab);
t = max(0, min(1, t));
proj = a + t*ab;
d = norm(p-proj);
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
