%% Task6/Task3 - Compare smoothing methods
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
clearance_req = 0.2;

read_only = struct();
read_only.map = map;
read_only.discrete_map = dmap;
read_only.mocap_pose = start_pose;
read_only.counter = 1;

base_public = struct();
base_public.path_clearance_m = clearance_req;

[raw_path, base_stats] = grid_planner_core(read_only, base_public, 'dijkstra');
if isempty(raw_path) || ~base_stats.success
    error('Failed to build base path for smoothing comparison.');
end

methods = {'iterative', 'shortcut', 'spline', 'chaikin'};
paths = cell(numel(methods), 1);
rows = cell(numel(methods), 1);

for i = 1:numel(methods)
    pv = base_public;
    pv.path_smoothing_mode = methods{i};

    t0 = tic;
    p = smooth_path(raw_path, read_only, pv);
    t = toc(t0);

    paths{i} = p;
    len = path_length(p);
    min_clear = path_min_clearance(p, map.walls, 0.03);
    smoothness = heading_variation(p);
    ok = ~isempty(p) && size(p,1) >= 2;
    clearance_ok = ok && (min_clear >= clearance_req);

    rows{i} = {methods{i}, ok, clearance_ok, size(p,1), len, min_clear, smoothness, t};
end

tbl = cell2table(vertcat(rows{:}), 'VariableNames', ...
    {'method','path_found','clearance_ok','waypoints','path_length_m','min_clearance_m','heading_variation_rad','runtime_s'});

% Lower is better for score.
len_n = tbl.path_length_m / max(tbl.path_length_m);
var_n = tbl.heading_variation_rad / max(tbl.heading_variation_rad);
time_n = tbl.runtime_s / max(tbl.runtime_s);
tbl.score = 0.5 * len_n + 0.4 * var_n + 0.1 * time_n;
[~, best_idx] = min(tbl.score);
best_method = tbl.method{best_idx};

writetable(tbl, fullfile(out_dir, 'task3_smoothing_comparison.csv'));

%% Figure
fig = figure('Name', 'Task6/Task3 smoothing comparison', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [90 90 1300 720]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile; hold on; axis equal; grid on;
for i = 1:size(map.walls, 1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 4, 'HandleVisibility', 'off');
end
plot(raw_path(:,1), raw_path(:,2), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Raw path');
styles = {'-','--',':','-.'};
colors = [0.10 0.45 0.95; 0.85 0.33 0.10; 0.20 0.70 0.25; 0.50 0.35 0.85];
for i = 1:numel(methods)
    p = paths{i};
    plot(p(:,1), p(:,2), styles{i}, 'Color', colors(i,:), 'LineWidth', 2.4, ...
        'DisplayName', sprintf('%s (min=%.3f m)', methods{i}, tbl.min_clearance_m(i)));
end
plot(start_pose(1), start_pose(2), 'kp', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Start');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 11, 'LineWidth', 2, 'DisplayName', 'Goal');
title('Path geometry');
xlabel('x [m]'); ylabel('y [m]');
axis([map.limits(1)-0.5 map.limits(3)+0.5 map.limits(2)-0.5 map.limits(4)+0.5]);
legend('Location', 'eastoutside', 'Interpreter', 'none');

nexttile;
catx = categorical(tbl.method);
yyaxis left;
b1 = bar(catx, [tbl.path_length_m, tbl.heading_variation_rad], 'grouped');
b1(1).DisplayName = 'Length [m]';
b1(2).DisplayName = 'Heading variation [rad]';
b1(1).FaceColor = [0.10 0.45 0.95];
b1(2).FaceColor = [0.85 0.33 0.10];
ylabel('Length [m] / Heading variation [rad]');

yyaxis right;
b2 = bar(catx, tbl.runtime_s, 0.28, 'FaceColor', [0.93 0.69 0.13], ...
    'EdgeColor', [0.35 0.25 0.05], 'DisplayName', 'Runtime [s]');
ylabel('Runtime [s]');

grid on;
title(sprintf('Metric comparison (best = %s)', best_method), 'FontWeight', 'bold');
legend([b1(1), b1(2), b2], 'Location', 'northeast');

png_path = fullfile(out_dir, 'task3_smoothing_comparison.png');
fig_path = fullfile(out_dir, 'task3_smoothing_comparison.fig');
savefig(fig, fig_path);
exportgraphics(fig, png_path, 'BackgroundColor', 'white', 'Resolution', 170);

%% Report
rep = fopen(fullfile(out_dir, 'task3_smoothing_report.md'), 'w');
fprintf(rep, '# Task6/Task3 - Porovnani vyhlazovacich metod\n\n');
fprintf(rep, '- Testovana mapa: `indoor_1`, start `[%.2f, %.2f]`, cil `[%.2f, %.2f]`\n', ...
    start_pose(1), start_pose(2), map.goal(1), map.goal(2));
fprintf(rep, '- Pozadovany odstup od prekazek: `>= %.2f m`\n\n', clearance_req);
fprintf(rep, '## Tabulka vysledku\n\n');
fprintf(rep, '| Metoda | Trasa nalezena | Odstup splnen | Waypointy | Delka [m] | Min odstup [m] | Sum zmen smeru [rad] | Cas [s] | Score |\n');
fprintf(rep, '|---|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(tbl)
    fprintf(rep, '| %s | %d | %d | %d | %.3f | %.3f | %.3f | %.4f | %.4f |\n', ...
        tbl.method{i}, tbl.path_found(i), tbl.clearance_ok(i), tbl.waypoints(i), ...
        tbl.path_length_m(i), tbl.min_clearance_m(i), tbl.heading_variation_rad(i), ...
        tbl.runtime_s(i), tbl.score(i));
end
fprintf(rep, '\n## Doporuceni\n');
fprintf(rep, '- Nejvhodnejsi metoda podle score: `%s`\n', best_method);
fprintf(rep, '- Vliv parametru:\n');
fprintf(rep, '  - Heading variation [rad] = soucet absolutnich zmen smeroveho uhlu podél trasy (mensi hodnota znamena plynulejsi trasu).\n');
fprintf(rep, '  - Iterative: vyssi `beta` vice vyhlazuje, vyssi `alpha` vice drzi puvodni trasu.\n');
fprintf(rep, '  - Shortcut: agresivne zkracuje trasu, ale muze ponechat ostrejsi zmeny smeru.\n');
fprintf(rep, '  - Spline: nejhladsi geometrie, ale muze se blizit prekazkam bez bezpecnostni kontroly.\n');
fprintf(rep, '  - Chaikin: plynule zaobluje rohy, pocet bodu roste s poctem iteraci.\n\n');
fprintf(rep, '## Vystupy\n');
fprintf(rep, '- `task3_smoothing_comparison.png`\n');
fprintf(rep, '- `task3_smoothing_comparison.csv`\n');
fprintf(rep, '- `task3_smoothing_report.md`\n');
fclose(rep);

fprintf('Saved: %s\n', rel_path(png_path, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task3_smoothing_comparison.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task3_smoothing_report.md'), project_root));

function L = path_length(p)
if isempty(p) || size(p,1) < 2
    L = inf;
    return;
end
L = sum(vecnorm(diff(p,1,1),2,2));
end

function v = heading_variation(p)
if isempty(p) || size(p,1) < 3
    v = inf;
    return;
end
d = diff(p, 1, 1);
psi = atan2(d(:,2), d(:,1));
v = sum(abs(wrap_to_pi_local(diff(psi))));
end

function dmin = path_min_clearance(path, walls, ds)
pts = sample_path(path, ds);
dmin = inf;
for i = 1:size(pts,1)
    pi = pts(i,:);
    for w = 1:size(walls,1)
        d = point_to_segment_distance(pi, walls(w,1:2), walls(w,3:4));
        if d < dmin
            dmin = d;
        end
    end
end
end

function pts = sample_path(path, ds)
pts = path(1,:);
for i = 1:(size(path,1)-1)
    a = path(i,:); b = path(i+1,:);
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
ab = b-a;
if dot(ab,ab) < 1e-12
    d = norm(p-a);
    return;
end
t = dot(p-a, ab)/dot(ab,ab);
t = max(0, min(1, t));
proj = a + t*ab;
d = norm(p-proj);
end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2*pi) - pi;
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
