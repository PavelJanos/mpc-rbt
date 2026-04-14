%% Task 3 - Task 3: Compare motion controllers on indoor_1 path
% Runs all plan_motion controller modes on the same predefined path.

clear; clc; close all;

project_root = find_project_root(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'utils'));
addpath(fullfile(project_root, 'algorithms'));
addpath(fullfile(project_root, 'algorithms', 'motion_control'));
addpath(fullfile(project_root, 'algorithms', 'path_planning'));

out_dir = fileparts(mfilename('fullpath'));
map = load_map(fullfile(project_root, 'maps', 'indoor_1.txt'));
map.goal_tolerance = 0.5;

start_pose = [2, 8.5, pi/2];
Ts = 0.1;
drive = struct('type', 2, 'interwheel_dist', 0.2, 'max_vel', 1.0);

base_read_only.map = map;
base_read_only.sampling_period = Ts;
base_read_only.agent_drive = drive;

public_init.path = plan_path(base_read_only, struct('path', []));
public_init.motion_vector = [0, 0];
public_init.estimated_pose = [];
public_init.particles = [];
public_init.init_iterations = 1;

methods = {'waypoint_p', 'pure_pursuit', 'cross_track_pd', 'stanley'};
trials_per_method = 20;
max_steps = 550;
collision_tol = 0.08;

rng(42);

rows = {};
traj = struct();

for m = 1:numel(methods)
    mode = methods{m};
    success = 0;
    collision = 0;
    out_of_bounds = 0;
    step_sum = 0;
    rmse_sum = 0;
    best_run_hist = [];
    best_run_err = inf;

    for t = 1:trials_per_method
        pose = start_pose;
        hist = nan(max_steps + 1, 3);
        hist(1, :) = pose;

        public_vars = public_init;
        public_vars.controller_mode = mode;

        reached = false;
        crashed = false;
        left_arena = false;
        per_step_err = nan(max_steps, 1);

        for k = 1:max_steps
            read_only_vars = base_read_only;
            read_only_vars.counter = k;
            read_only_vars.mocap_pose = pose;

            public_vars = plan_motion(read_only_vars, public_vars);
            pose = move_agent(pose, public_vars.motion_vector, drive, Ts);
            hist(k + 1, :) = pose;

            if is_outside_limits(pose(1:2), map.limits)
                left_arena = true;
                break;
            end
            if is_collision_with_walls(pose(1:2), map.walls, collision_tol)
                crashed = true;
                break;
            end

            per_step_err(k) = dist_point_to_polyline(pose(1:2), public_vars.path);

            if norm(pose(1:2) - map.goal) <= map.goal_tolerance
                reached = true;
                break;
            end
        end

        valid_err = per_step_err(~isnan(per_step_err));
        if isempty(valid_err)
            rmse = nan;
        else
            rmse = sqrt(mean(valid_err .^ 2));
        end

        if reached
            success = success + 1;
            step_sum = step_sum + find(~isnan(hist(:, 1)), 1, 'last') - 1;
        end
        if crashed
            collision = collision + 1;
        end
        if left_arena
            out_of_bounds = out_of_bounds + 1;
        end
        if ~isnan(rmse)
            rmse_sum = rmse_sum + rmse;
            if rmse < best_run_err && reached
                best_run_err = rmse;
                best_run_hist = hist;
            end
        end
    end

    success_rate = 100 * success / trials_per_method;
    if success > 0
        avg_steps_success = step_sum / success;
    else
        avg_steps_success = nan;
    end
    avg_rmse = rmse_sum / trials_per_method;

    rows(end + 1, :) = {mode, success_rate, avg_steps_success, avg_rmse, collision, out_of_bounds}; %#ok<AGROW>
    traj.(mode) = best_run_hist;
end

summary = cell2table(rows, 'VariableNames', ...
    {'mode', 'success_rate_pct', 'avg_steps_success', 'avg_path_rmse_m', 'collision_count', 'out_count'});

writetable(summary, fullfile(out_dir, 'task3_motion_methods_summary.csv'));
save(fullfile(out_dir, 'task3_motion_methods_results.mat'), 'summary', 'traj', 'methods', 'trials_per_method');

plot_results(map, public_init.path, traj, methods, out_dir);

print_summary(summary, out_dir, methods, trials_per_method);

fprintf('\nDone. Outputs saved in: %s\n', rel_path(out_dir, project_root));

function root = find_project_root(start_dir)
root = start_dir;
while ~exist(fullfile(root, 'main.m'), 'file')
    parent = fileparts(root);
    if strcmp(parent, root)
        error('Could not locate project root (main.m).');
    end
    root = parent;
end
end

function tf = is_outside_limits(point, limits)
tf = point(1) < limits(1) || point(1) > limits(3) || point(2) < limits(2) || point(2) > limits(4);
end

function tf = is_collision_with_walls(point, walls, tol)
tf = false;
for i = 1:size(walls, 1)
    d = dist_point_to_segment(point, walls(i, 1:2), walls(i, 3:4));
    if d <= tol
        tf = true;
        return;
    end
end
end

function d = dist_point_to_polyline(point, polyline)
if size(polyline, 1) < 2
    d = inf;
    return;
end
d = inf;
for i = 1:(size(polyline, 1) - 1)
    d = min(d, dist_point_to_segment(point, polyline(i, :), polyline(i + 1, :)));
end
end

function d = dist_point_to_segment(point, a, b)
ab = b - a;
if all(abs(ab) < 1e-12)
    d = norm(point - a);
    return;
end
t = dot(point - a, ab) / dot(ab, ab);
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(point - proj);
end

function plot_results(map, path, traj, methods, out_dir)
fig = figure('Name', 'Task3 Task3 Controller Comparison', 'NumberTitle', 'off', 'Position', [120, 120, 1100, 700]);
hold on; axis equal; grid on;

for i = 1:size(map.walls, 1)
    line([map.walls(i, 1), map.walls(i, 3)], [map.walls(i, 2), map.walls(i, 4)], ...
        'Color', 'k', 'LineWidth', 4, 'HandleVisibility', 'off');
end

h_ref = plot(path(:, 1), path(:, 2), 'Color', [0.1 0.7 0.1], 'LineWidth', 2.5, 'DisplayName', 'Reference path');
h_goal = plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');

legend_handles = [h_ref, h_goal];
legend_labels = {'Reference path', 'Goal'};

colors = lines(numel(methods));
for i = 1:numel(methods)
    mode = methods{i};
    h = traj.(mode);
    if ~isempty(h)
        valid = ~isnan(h(:, 1));
        h_mode = plot(h(valid, 1), h(valid, 2), '-', 'Color', colors(i, :), 'LineWidth', 1.8, 'DisplayName', mode);
        legend_handles(end + 1) = h_mode; %#ok<AGROW>
        legend_labels{end + 1} = mode; %#ok<AGROW>
    end
end

title('Task3/Task3: Reference path vs best trajectories per controller');
xlabel('x [m]'); ylabel('y [m]');
legend(legend_handles, legend_labels, 'Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5 map.limits(3)+0.5 map.limits(2)-0.5 map.limits(4)+0.5]);

savefig(fig, fullfile(out_dir, 'task3_motion_methods_comparison.fig'));
saveas(fig, fullfile(out_dir, 'task3_motion_methods_comparison.png'));
end

function print_summary(summary, out_dir, methods, trials)
report = fopen(fullfile(out_dir, 'task3_task3_report.md'), 'w');
if report < 0
    error('Cannot write report file.');
end

fprintf(report, '# Task3 - Task3: Srovnani metod rizeni trasy\n\n');
fprintf(report, 'Testovano na mape `indoor_1` se startem `(2, 8.5)` a MoCap polohou.\n\n');
fprintf(report, '- Pocet pokusu na metodu: %d\n', trials);
fprintf(report, '- Maximalni delka behu: 550 kroku\n');
fprintf(report, '- Metody: `%s`, `%s`, `%s`, `%s`\n\n', methods{1}, methods{2}, methods{3}, methods{4});

fprintf(report, '## Vysledky\n\n');
fprintf(report, '| Metoda | Uspesnost [%%] | Prum. kroky (jen uspech) | RMSE od trasy [m] | Kolize [pocet] | Mimo mapu [pocet] |\n');
fprintf(report, '|---|---:|---:|---:|---:|---:|\n');
for i = 1:height(summary)
    fprintf(report, '| %s | %.1f | %.1f | %.3f | %d | %d |\n', ...
        summary.mode{i}, summary.success_rate_pct(i), summary.avg_steps_success(i), ...
        summary.avg_path_rmse_m(i), summary.collision_count(i), summary.out_count(i));
end

fprintf(report, '\n## Diskuze parametru metody\n\n');
fprintf(report, '- `waypoint_p`: hlavni parametry jsou `K_heading`, `v_nom`, `wp_tol`. Vyssi `K_heading` zrychli nataceni, ale muze zpusobit kmitani.\n');
fprintf(report, '- `pure_pursuit`: hlavni parametry jsou `lookahead_dist`, `v_nom`, `k_curve_slow`. Mensi lookahead zvysuje presnost, ale zhorsuje stabilitu.\n');
fprintf(report, '- `cross_track_pd`: `K_cte`, `K_heading`, `K_d` urcuji kompromis mezi rychlou korekci a prekmitanim.\n');
fprintf(report, '- `stanley`: `k_stanley`, `K_w`, `v_soft` ovlivnuji citlivost na pricnou chybu. Vyssi `k_stanley` drzi trasu tesneji, ale byva mene hladky.\n');

fprintf(report, '\n## Vystupy\n\n');
fprintf(report, '- `task3_motion_methods_summary.csv`\n');
fprintf(report, '- `task3_motion_methods_comparison.png`\n');
fprintf(report, '- `task3_motion_methods_results.mat`\n');

fclose(report);
end

function rp = rel_path(path_abs, root_abs)
rp = strrep(path_abs, [root_abs filesep], '');
end
