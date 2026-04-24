%% Task6/Task1 - Compare grid path planners
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

% Test map for Task6 planning comparison.
map = load_map(fullfile(project_root, 'maps', 'indoor_1.txt'));
map.discretization_step = 0.2;
discrete_map = generate_discrete_map(map);

start_pose = [2.0, 1.0, -pi/2];
goal_xy = map.goal;

read_only_vars = struct();
read_only_vars.map = map;
read_only_vars.discrete_map = discrete_map;
read_only_vars.mocap_pose = start_pose;
read_only_vars.counter = 1;

planners = {'astar', 'dijkstra', 'greedy'};
rows = cell(numel(planners), 1);
paths = cell(numel(planners), 1);

for i = 1:numel(planners)
    public_vars = struct();
    public_vars.path = [];
    public_vars.force_grid_planner = true;
    public_vars.path_planner_mode = planners{i};
    public_vars.replan_path = true;

    t0 = tic;
    p = plan_path(read_only_vars, public_vars);
    t = toc(t0);

    ok = ~isempty(p) && size(p, 1) >= 2;
    if ok
        plen = sum(vecnorm(diff(p, 1, 1), 2, 2));
        goal_err = norm(p(end, :) - goal_xy);
    else
        plen = inf;
        goal_err = inf;
    end

    rows{i} = {planners{i}, ok, size(p, 1), plen, goal_err, t};
    paths{i} = p;
end

tbl = cell2table(vertcat(rows{:}), 'VariableNames', ...
    {'planner', 'success', 'waypoints', 'path_length_m', 'goal_error_m', 'runtime_s'});
tbl.score = inf(height(tbl), 1);

valid = tbl.success;
if any(valid)
    len_norm = tbl.path_length_m(valid) / max(tbl.path_length_m(valid));
    time_norm = tbl.runtime_s(valid) / max(tbl.runtime_s(valid));
    tbl.score(valid) = 0.7 * len_norm + 0.3 * time_norm;
end

[~, best_idx] = min(tbl.score);
best_planner = tbl.planner{best_idx};
writetable(tbl, fullfile(out_dir, 'task1_planner_comparison.csv'));

%% Figure
fig = figure('Name', 'Task6/Task1 planner comparison', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [120 120 1200 700]);
hold on; axis equal; grid on;

for i = 1:size(map.walls, 1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 4, 'HandleVisibility', 'off');
end

line([map.limits(1), map.limits(3)], [map.limits(2), map.limits(2)], 'Color','black', 'LineWidth',1, 'LineStyle', '--', 'HandleVisibility', 'off');
line([map.limits(3), map.limits(3)], [map.limits(2), map.limits(4)], 'Color','black', 'LineWidth',1, 'LineStyle', '--', 'HandleVisibility', 'off');
line([map.limits(1), map.limits(3)], [map.limits(4), map.limits(4)], 'Color','black', 'LineWidth',1, 'LineStyle', '--', 'HandleVisibility', 'off');
line([map.limits(1), map.limits(1)], [map.limits(2), map.limits(4)], 'Color','black', 'LineWidth',1, 'LineStyle', '--', 'HandleVisibility', 'off');

styles = {'-','--',':'};
colors = [0.10 0.45 0.95; 0.85 0.33 0.10; 0.20 0.70 0.25];
for i = 1:numel(planners)
    p = paths{i};
    if ~isempty(p)
        plot(p(:,1), p(:,2), styles{i}, 'Color', colors(i,:), 'LineWidth', 2.5, ...
            'DisplayName', sprintf('%s', planners{i}));
    end
end

plot(start_pose(1), start_pose(2), 'kp', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Start');
plot(goal_xy(1), goal_xy(2), 'go', 'MarkerSize', 11, 'LineWidth', 2, 'DisplayName', 'Goal');

title(sprintf('Task6/Task1: planner comparison (best = %s)', best_planner), 'FontWeight', 'bold');
xlabel('x [m]');
ylabel('y [m]');
legend('Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5, map.limits(3)+0.5, map.limits(2)-0.5, map.limits(4)+0.5]);

png_path = fullfile(out_dir, 'task1_planner_comparison.png');
fig_path = fullfile(out_dir, 'task1_planner_comparison.fig');
savefig(fig, fig_path);
exportgraphics(fig, png_path, 'BackgroundColor', 'white', 'Resolution', 170);

%% Report
rep = fopen(fullfile(out_dir, 'task1_planner_report.md'), 'w');
fprintf(rep, '# Task6/Task1 - Porovnani planovacich algoritmu\n\n');
fprintf(rep, 'Pouzite algoritmy: `A*`, `Dijkstra`, `Greedy Best-First`.\n\n');
fprintf(rep, 'Mapa: `indoor_1`, start `[%.2f, %.2f]`, cil `[%.2f, %.2f]`.\n\n', ...
    start_pose(1), start_pose(2), goal_xy(1), goal_xy(2));
fprintf(rep, '## Vysledky\n\n');
fprintf(rep, '| Algoritmus | Uspech | Pocet waypointu | Delka trasy [m] | Chyba k cili [m] | Cas [s] | Score |\n');
fprintf(rep, '|---|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(tbl)
    fprintf(rep, '| %s | %d | %d | %.3f | %.3f | %.4f | %.4f |\n', ...
        tbl.planner{i}, tbl.success(i), tbl.waypoints(i), tbl.path_length_m(i), ...
        tbl.goal_error_m(i), tbl.runtime_s(i), tbl.score(i));
end

fprintf(rep, '\n## Doporuceny default\n');
fprintf(rep, '- Doporuceny planner: `%s`\n', best_planner);
fprintf(rep, '- Duvod: nejlepsi kompromis delky trasy a vypocetniho casu pri uspesnem nalezeni cesty.\n\n');
fprintf(rep, '## Jak prepinat planner\n');
fprintf(rep, 'V `public_vars` nastavte:\n\n');
fprintf(rep, '```matlab\n');
fprintf(rep, 'public_vars.force_grid_planner = true;\n');
fprintf(rep, 'public_vars.path_planner_mode = ''astar'';    %% nebo ''dijkstra'' / ''greedy''\n');
fprintf(rep, '```\n');
fclose(rep);

fprintf('Saved: %s\n', rel_path(png_path, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task1_planner_comparison.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task1_planner_report.md'), project_root));

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
