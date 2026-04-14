%% Task5/Task3 - EKF tuning with known initial pose (separate R terms)
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
addpath(fullfile(project_root, 'algorithms', 'motion_control'));
addpath(fullfile(project_root, 'algorithms', 'path_planning'));
addpath(fullfile(project_root, 'algorithms', 'kalman_filter'));

out_dir = fileparts(mfilename('fullpath'));
rng(41);

%% Common setup
map = load_map(fullfile(project_root, 'maps', 'outdoor_1.txt'));
map.goal_tolerance = 0.5;
Ts = 0.1;
drive = struct('type', 2, 'interwheel_dist', 0.2, 'max_vel', 1.0);
start_pose = [2, 2, pi/2];
repeats = 4;

R_best = [0.01, 0.01, 0.01]; % assignment baseline
all_rows = [];
best_runs = struct();

% Tune each diagonal element separately: R_xx, R_yy, R_tt
params = {'R_xx', 'R_yy', 'R_tt'};
for p = 1:3
    % Coarse sweep around baseline.
    coarse_vals = [0.003, 0.005, 0.008, 0.010, 0.015, 0.020, 0.030];
    [tbl_c, best_c, run_c] = sweep_param(p, coarse_vals, R_best, repeats, ...
        map, drive, Ts, start_pose, project_root, "coarse", params{p});

    % Fine sweep around best coarse value.
    v0 = best_c.value;
    fine_vals = unique(max(v0 * [0.70, 0.85, 0.95, 1.00, 1.05, 1.15, 1.30], 1e-4));
    [tbl_f, best_f, run_f] = sweep_param(p, fine_vals, R_best, repeats, ...
        map, drive, Ts, start_pose, project_root, "fine", params{p});

    % Keep best fine value for next dimension.
    R_best(p) = best_f.value;

    all_rows = [all_rows; tbl_c; tbl_f]; %#ok<AGROW>
    best_runs.(params{p}) = run_f;
end

% Final robust evaluation with jointly tuned R
R_final = diag(R_best);
final_data = simulate_ekf_run(map, drive, Ts, start_pose, R_final, project_root, 999);

all_rows.quality_score = 1 ./ (1 + all_rows.cost_score); % higher is better
writetable(all_rows, fullfile(out_dir, 'task3_ekf_tuning_results.csv'));
writetable(all_rows, fullfile(out_dir, 'task3_R_tuning_table.csv'));

%% Figure 1: final run
fig1 = figure('Name', 'Task5/Task3 EKF final run', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [90 90 1260 640]);
t1 = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile([2 1]); hold on; axis equal; grid on;
for i = 1:size(map.walls, 1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 3, 'HandleVisibility', 'off');
end
plot(final_data.path(:,1), final_data.path(:,2), '-', 'Color', [0.15 0.68 0.22], 'LineWidth', 2.2, 'DisplayName', 'Reference path');
plot(final_data.true_hist(:,1), final_data.true_hist(:,2), '-', 'Color', [0.15 0.35 0.95], 'LineWidth', 1.8, 'DisplayName', 'True trajectory');
plot(final_data.ekf_hist(:,1), final_data.ekf_hist(:,2), '--', 'Color', [0.88 0.22 0.22], 'LineWidth', 1.9, 'DisplayName', 'EKF estimate');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');
title(sprintf('Final EKF run (R=[%.5f %.5f %.5f])', R_best(1), R_best(2), R_best(3)));
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5 map.limits(3)+0.5 map.limits(2)-0.5 map.limits(4)+0.5]);

nexttile;
plot(final_data.gnss_err, 'Color', [0.45 0.45 0.45], 'LineWidth', 1.3, 'DisplayName', 'GNSS XY error'); hold on;
plot(final_data.ekf_err, 'Color', [0.88 0.22 0.22], 'LineWidth', 1.8, 'DisplayName', 'EKF XY error');
grid on; xlabel('Step'); ylabel('Error [m]');
title(sprintf('XY errors (RMSE GNSS %.3f, EKF %.3f)', final_data.rmse_gnss_xy, final_data.rmse_ekf_xy));
legend('Location', 'best');

nexttile;
plot(final_data.theta_err, 'Color', [0.55 0.25 0.82], 'LineWidth', 1.6);
grid on; xlabel('Step'); ylabel('|e_\\theta| [rad]');
title(sprintf('Heading error (mean %.3f rad)', final_data.mean_theta_err));
title(t1, 'Task5/Task3: Final EKF run after separate R tuning', 'FontWeight', 'bold');

png_run = fullfile(out_dir, 'task3_known_start_ekf_result.png');
fig_run = fullfile(out_dir, 'task3_known_start_ekf_result.fig');
savefig(fig1, fig_run);
exportgraphics(fig1, png_run, 'BackgroundColor', 'white', 'Resolution', 170);

%% Figure 2: tuning progression per parameter
fig2 = figure('Name', 'Task5/Task3 R tuning progress', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [100 100 1250 780]);
t2 = tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

for p = 1:3
    nexttile;
    key = params{p};
    idx_p = strcmp(all_rows.param, key);
    c = idx_p & strcmp(all_rows.phase, "coarse");
    f = idx_p & strcmp(all_rows.phase, "fine");

    plot(all_rows.test_value(c), all_rows.quality_score(c), 'x--', 'LineWidth', 1.6, ...
        'Color', [0.20 0.45 0.90], 'DisplayName', [key ' coarse']); hold on;
    plot(all_rows.test_value(f), all_rows.quality_score(f), '^--', 'LineWidth', 1.6, ...
        'Color', [0.10 0.70 0.35], 'DisplayName', [key ' fine']);
    [mx, imx] = max(all_rows.quality_score(idx_p));
    xv = all_rows.test_value(idx_p);
    xv = xv(imx);
    plot(xv, mx, 'ro', 'MarkerSize', 8, 'LineWidth', 1.8, 'DisplayName', 'best');
    grid on;
    xlabel([key ' candidate value']);
    ylabel('Quality score');
    title([key ' tuning']);
    legend('Location', 'best');
end

title(t2, 'Separate coarse-to-fine tuning of R diagonal terms', 'FontWeight', 'bold');

png_tune = fullfile(out_dir, 'task3_R_tuning_progress.png');
fig_tune = fullfile(out_dir, 'task3_R_tuning_progress.fig');
savefig(fig2, fig_tune);
exportgraphics(fig2, png_tune, 'BackgroundColor', 'white', 'Resolution', 170);

%% Report
rep = fopen(fullfile(out_dir, 'task3_known_start_report.md'), 'w');
fprintf(rep, '# Task5/Task3 - Ladeni filtru se znamou pocatecni polohou (separate R)\n\n');
fprintf(rep, '- Start: `[2,2,pi/2]`\n');
fprintf(rep, '- Pocatecni vira: `mu=[2,2,pi/2]`, `Sigma=zeros(3,3)`\n');
fprintf(rep, '- Q: prevzata z Task5/Task1 (`task1_gnss_init_data.mat`), fallback `diag([0.25,0.25])`\n');
fprintf(rep, '- Opakovani na kandidata: %d\n', repeats);
fprintf(rep, '- Tuning: zvlast pro `R_xx`, `R_yy`, `R_tt` (coarse + fine)\n\n');

fprintf(rep, '## Nejlepsi nastaveni R\n');
fprintf(rep, '- `R = diag([%.6f, %.6f, %.6f])`\n', R_best(1), R_best(2), R_best(3));
fprintf(rep, '- Final run reached goal: %d\n', final_data.reached_goal);
fprintf(rep, '- Final run steps: %d\n', final_data.steps);
fprintf(rep, '- RMSE GNSS (XY): %.4f m\n', final_data.rmse_gnss_xy);
fprintf(rep, '- RMSE EKF (XY): %.4f m\n', final_data.rmse_ekf_xy);
fprintf(rep, '- Improvement EKF vs GNSS: %.1f %%\n', ...
    100 * (final_data.rmse_gnss_xy - final_data.rmse_ekf_xy) / max(final_data.rmse_gnss_xy, 1e-9));
fprintf(rep, '- Mean |e_theta|: %.4f rad\n\n', final_data.mean_theta_err);

fprintf(rep, '## Vystupy\n');
fprintf(rep, '- `task3_known_start_ekf_result.png`\n');
fprintf(rep, '- `task3_R_tuning_progress.png`\n');
fprintf(rep, '- `task3_R_tuning_table.csv`\n');
fprintf(rep, '- `task3_ekf_tuning_results.csv`\n');
fprintf(rep, '- `task3_known_start_report.md`\n');
fclose(rep);

fprintf('Saved: %s\n', rel_path(png_run, project_root));
fprintf('Saved: %s\n', rel_path(png_tune, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task3_R_tuning_table.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task3_known_start_report.md'), project_root));

function [tbl, best, best_run] = sweep_param(p_idx, values, R_base, repeats, map, drive, Ts, start_pose, project_root, phase_name, param_name)
rows = [];
best.cost = inf;
best.value = values(1);
best_run = [];

for i = 1:numel(values)
    v = values(i);
    R_vec = R_base;
    R_vec(p_idx) = v;
    R = diag(R_vec);

    runs = cell(repeats, 1);
    best_local_cost = inf;
    best_local_run = [];

    for r = 1:repeats
        seed = 1000 + 100*p_idx + 10*i + r;
        run_data = simulate_ekf_run(map, drive, Ts, start_pose, R, project_root, seed);
        runs{r} = run_data;
        c = (1 - run_data.reached_goal) * 1e3 + run_data.rmse_ekf_xy;
        if c < best_local_cost
            best_local_cost = c;
            best_local_run = run_data;
        end
    end

    reached = cellfun(@(x) x.reached_goal, runs);
    steps = cellfun(@(x) x.steps, runs);
    rmse_g = cellfun(@(x) x.rmse_gnss_xy, runs);
    rmse_e = cellfun(@(x) x.rmse_ekf_xy, runs);
    theta_e = cellfun(@(x) x.mean_theta_err, runs);

    success_rate = mean(reached);
    if any(reached > 0)
        mean_steps_success = mean(steps(reached > 0));
    else
        mean_steps_success = nan;
    end

    mean_rmse_ekf = mean(rmse_e);
    mean_rmse_gnss = mean(rmse_g);
    mean_theta = mean(theta_e);
    cost_score = (1 - success_rate) * 1e3 + mean_rmse_ekf;

    rows = [rows; i, v, success_rate, mean_steps_success, mean_rmse_gnss, mean_rmse_ekf, mean_theta, cost_score]; %#ok<AGROW>

    if cost_score < best.cost
        best.cost = cost_score;
        best.value = v;
        best.success_rate = success_rate;
        best.mean_steps_success = mean_steps_success;
        best.mean_rmse_gnss = mean_rmse_gnss;
        best.mean_rmse_ekf = mean_rmse_ekf;
        best.mean_theta_err = mean_theta;
        best_run = best_local_run;
    end
end

tbl = array2table(rows, 'VariableNames', ...
    {'id','test_value','success_rate','mean_steps_success','mean_rmse_gnss_xy','mean_rmse_ekf_xy','mean_theta_err','cost_score'});
tbl.phase = repmat(string(phase_name), height(tbl), 1);
tbl.param = repmat(string(param_name), height(tbl), 1);
tbl = movevars(tbl, {'param','phase'}, 'Before', 'id');
end

function data = simulate_ekf_run(map, drive, Ts, start_pose, R, project_root, seed)
if nargin >= 7
    rng(seed);
end

read_only.map = map;
read_only.sampling_period = Ts;
read_only.agent_drive = drive;

true_pose = start_pose;
read_only.mocap_pose = true_pose;

public_vars = struct();
public_vars.motion_vector = [0, 0];
public_vars.path = plan_path(read_only, struct('path', []));
public_vars.controller_mode = 'pure_pursuit';
public_vars.use_estimated_pose_only = true;

public_vars = init_kalman_filter(read_only, public_vars);
public_vars.mu = start_pose(:);
public_vars.sigma = zeros(3, 3);
public_vars.kf.R = R;

gnss_init_file = fullfile(project_root, 'algorithms', 'tasks', 'Task5', 'Task1', 'task1_gnss_init_data.mat');
if isfile(gnss_init_file)
    s = load(gnss_init_file, 'gnss_sigma');
    if isfield(s, 'gnss_sigma') && all(size(s.gnss_sigma) == [2, 2])
        public_vars.kf.Q = s.gnss_sigma;
    end
end

max_steps = 900;
true_hist = nan(max_steps, 3);
ekf_hist = nan(max_steps, 3);
gnss_hist = nan(max_steps, 2);

reached = false;
for k = 1:max_steps
    if norm(true_pose(1:2) - map.goal) <= map.goal_tolerance
        reached = true;
        true_hist(k:end,:) = [];
        ekf_hist(k:end,:) = [];
        gnss_hist(k:end,:) = [];
        break;
    end

    read_only.counter = k;
    read_only.mocap_pose = true_pose;
    read_only.gnss_position = gnss_measure(true_pose, map.gnss_denied);

    public_vars.estimated_pose = public_vars.mu(:)';
    public_vars = plan_motion(read_only, public_vars);

    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only, public_vars);
    true_pose = move_agent(true_pose, public_vars.motion_vector, drive, Ts);

    true_hist(k,:) = true_pose;
    ekf_hist(k,:) = public_vars.mu(:)';
    gnss_hist(k,:) = read_only.gnss_position;
end

valid = ~isnan(true_hist(:,1));
true_hist = true_hist(valid,:);
ekf_hist = ekf_hist(valid,:);
gnss_hist = gnss_hist(valid,:);

gnss_err = vecnorm(gnss_hist - true_hist(:,1:2), 2, 2);
ekf_err = vecnorm(ekf_hist(:,1:2) - true_hist(:,1:2), 2, 2);
theta_err = abs(wrap_to_pi_local(ekf_hist(:,3) - true_hist(:,3)));

data.path = public_vars.path;
data.true_hist = true_hist;
data.ekf_hist = ekf_hist;
data.gnss_hist = gnss_hist;
data.gnss_err = gnss_err;
data.ekf_err = ekf_err;
data.theta_err = theta_err;
data.rmse_gnss_xy = sqrt(mean(gnss_err.^2, 'omitnan'));
data.rmse_ekf_xy = sqrt(mean(ekf_err.^2, 'omitnan'));
data.mean_theta_err = mean(theta_err, 'omitnan');
data.steps = size(true_hist,1);
data.reached_goal = reached;
end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2*pi) - pi;
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
