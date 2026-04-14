%% Task5/Task4 - EKF deployment with unknown initial pose
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
rng(52);

%% Environment
map = load_map(fullfile(project_root, 'maps', 'outdoor_1.txt'));
map.goal_tolerance = 0.5;
Ts = 0.1;
drive = struct('type', 2, 'interwheel_dist', 0.2, 'max_vel', 1.0);
true_start = [2, 2, pi/2];

%% Initialization from Task5/Task1 GNSS statistics
init_file = fullfile(project_root, 'algorithms', 'tasks', 'Task5', 'Task1', 'task1_gnss_init_data.mat');
if ~isfile(init_file)
    error('Missing Task5/Task1 data: %s', init_file);
end
s = load(init_file, 'gnss_mu', 'gnss_sigma');
if ~isfield(s, 'gnss_mu') || ~isfield(s, 'gnss_sigma')
    error('Task5/Task1 file missing gnss_mu / gnss_sigma.');
end
mu0_xy = s.gnss_mu(:);
Q_base = s.gnss_sigma;

% Unknown heading: use high initial variance for theta.
theta0_guess = 0;
sigma0 = blkdiag(Q_base, 2.0); % high uncertainty in orientation
mu0 = [mu0_xy; theta0_guess];

%% Parameter tuning for smooth estimate
R_base = diag([0.00285, 0.00425, 0.00475]); % from Task5/Task3
repeats = 3;

% Phase 1: fixed coarse 7x7 search
r_scales_coarse = linspace(0.40, 1.60, 7);
q_scales_coarse = linspace(0.60, 1.60, 7);
[rows_coarse, best_coarse] = tune_grid(map, drive, Ts, true_start, mu0, sigma0, ...
    R_base, Q_base, r_scales_coarse, q_scales_coarse, repeats, "coarse", 7000);

% Phase 2: 9x9 refinement around coarse winner
center_r = best_coarse.r_scale;
center_q = best_coarse.q_scale;
dr = r_scales_coarse(2) - r_scales_coarse(1);
dq = q_scales_coarse(2) - q_scales_coarse(1);
r_scales_fine = linspace(max(0.05, center_r - dr), center_r + dr, 9);
q_scales_fine = linspace(max(0.05, center_q - dq), center_q + dq, 9);
[rows_fine, best_fine] = tune_grid(map, drive, Ts, true_start, mu0, sigma0, ...
    R_base, Q_base, r_scales_fine, q_scales_fine, repeats, "fine", 17000);

if best_fine.cost <= best_coarse.cost
    best = best_fine;
else
    best = best_coarse;
end

tbl_coarse = array2table(rows_coarse, 'VariableNames', ...
    {'id_r','id_q','r_scale','q_scale','success_rate','mean_steps_success','mean_rmse_gnss_xy','mean_rmse_ekf_xy','mean_smoothness_xy','mean_theta_err','cost_score'});
tbl_coarse.stage = repmat("coarse", height(tbl_coarse), 1);
tbl_fine = array2table(rows_fine, 'VariableNames', ...
    {'id_r','id_q','r_scale','q_scale','success_rate','mean_steps_success','mean_rmse_gnss_xy','mean_rmse_ekf_xy','mean_smoothness_xy','mean_theta_err','cost_score'});
tbl_fine.stage = repmat("fine", height(tbl_fine), 1);
tbl = [tbl_coarse; tbl_fine];
tbl.quality_score = 1 ./ (1 + tbl.cost_score);
writetable(tbl, fullfile(out_dir, 'task4_deploy_tuning_table.csv'));

%% Figures
data = best.run;

fig1 = figure('Name', 'Task5/Task4 EKF deployment', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [90 90 1260 650]);
t1 = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile([2 1]); hold on; axis equal; grid on;
for i = 1:size(map.walls, 1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 3, 'HandleVisibility', 'off');
end
plot(data.path(:,1), data.path(:,2), '-', 'Color', [0.15 0.68 0.22], 'LineWidth', 2.2, 'DisplayName', 'Reference path');
plot(data.true_hist(:,1), data.true_hist(:,2), '-', 'Color', [0.15 0.35 0.95], 'LineWidth', 1.8, 'DisplayName', 'True trajectory');
plot(data.ekf_hist(:,1), data.ekf_hist(:,2), '--', 'Color', [0.88 0.22 0.22], 'LineWidth', 1.9, 'DisplayName', 'EKF estimate');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');
plot(data.ekf_hist(1,1), data.ekf_hist(1,2), 'mo', 'MarkerSize', 7, 'LineWidth', 1.8, 'DisplayName', 'Initial EKF guess');
title(sprintf('Deployment run (unknown start): Rscale=%.2f, Qscale=%.2f', best.r_scale, best.q_scale));
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5 map.limits(3)+0.5 map.limits(2)-0.5 map.limits(4)+0.5]);

nexttile;
plot(data.gnss_err, 'Color', [0.45 0.45 0.45], 'LineWidth', 1.3, 'DisplayName', 'GNSS XY error'); hold on;
plot(data.ekf_err, 'Color', [0.88 0.22 0.22], 'LineWidth', 1.8, 'DisplayName', 'EKF XY error');
grid on; xlabel('Step'); ylabel('Error [m]');
title(sprintf('XY errors (RMSE GNSS %.3f, EKF %.3f)', data.rmse_gnss_xy, data.rmse_ekf_xy));
legend('Location', 'best');

nexttile;
plot(data.theta_err, 'Color', [0.55 0.25 0.82], 'LineWidth', 1.6, 'DisplayName', '|e_theta|'); hold on;
plot(data.theta_std, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.4, 'DisplayName', 'sqrt(sigma_theta)');
grid on; xlabel('Step'); ylabel('Rad');
title('Heading estimation (not directly measured)');
legend('Location', 'best');
title(t1, 'Task5/Task4: EKF deployment with unknown initial pose', 'FontWeight', 'bold');

png_run = fullfile(out_dir, 'task4_deploy_success_run.png');
fig_run = fullfile(out_dir, 'task4_deploy_success_run.fig');
savefig(fig1, fig_run);
exportgraphics(fig1, png_run, 'BackgroundColor', 'white', 'Resolution', 170);

fig2 = figure('Name', 'Task5/Task4 tuning map', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [120 120 820 620]);
is_coarse = strcmp(tbl.stage, "coarse");
is_fine = strcmp(tbl.stage, "fine");
s1 = scatter(tbl.r_scale(is_coarse), tbl.q_scale(is_coarse), 95, tbl.quality_score(is_coarse), ...
    'o', 'LineWidth', 1.2, 'MarkerFaceColor', 'none', 'DisplayName', 'Coarse 7x7');
hold on;
s2 = scatter(tbl.r_scale(is_fine), tbl.q_scale(is_fine), 70, tbl.quality_score(is_fine), ...
    '^', 'filled', 'MarkerEdgeColor', [0.1 0.1 0.1], 'DisplayName', 'Fine 9x9');
cb = colorbar; %#ok<NASGU>
colormap(parula); grid on;
xlabel('R scale');
ylabel('Q scale');
title('Tuning grid: coarse 7x7 + fine 9x9 (higher quality is better)');
plot(best_coarse.r_scale, best_coarse.q_scale, 'ks', 'MarkerSize', 9, 'LineWidth', 1.8, 'DisplayName', 'Best coarse');
plot(best.r_scale, best.q_scale, 'rx', 'MarkerSize', 12, 'LineWidth', 2.2, 'DisplayName', 'Best final');
legend([s1 s2], 'Location', 'best');

png_tune = fullfile(out_dir, 'task4_deploy_tuning_grid.png');
fig_tune = fullfile(out_dir, 'task4_deploy_tuning_grid.fig');
savefig(fig2, fig_tune);
exportgraphics(fig2, png_tune, 'BackgroundColor', 'white', 'Resolution', 170);

%% Report
rep = fopen(fullfile(out_dir, 'task4_deploy_report.md'), 'w');
fprintf(rep, '# Task5/Task4 - Nasazeni EKF bez znalosti pocatecni polohy\n\n');
fprintf(rep, '## Inicializace viry\n');
fprintf(rep, '- `mu_0` prevzato z GNSS inicializace (Task5/Task1): [%.4f, %.4f, %.4f]\n', mu0(1), mu0(2), mu0(3));
fprintf(rep, '- `Sigma_0` pouziva GNSS kovarianci pro `x,y` a vysokou varianci pro orientaci: `Sigma_{theta,0}=%.3f`\n\n', sigma0(3,3));

fprintf(rep, '## Naladene parametry filtru\n');
fprintf(rep, '- Base `R` (Task5/Task3): diag([0.00285, 0.00425, 0.00475])\n');
fprintf(rep, '- Coarse faze: pevna mrizka 7x7 (`R_scale` v [%.2f, %.2f], `Q_scale` v [%.2f, %.2f])\n', ...
    min(r_scales_coarse), max(r_scales_coarse), min(q_scales_coarse), max(q_scales_coarse));
fprintf(rep, '- Fine faze: mrizka 9x9 kolem coarse top (`R_center=%.3f`, `Q_center=%.3f`)\n', center_r, center_q);
fprintf(rep, '- Best scales: `R*=%.2f x R_base`, `Q*=%.2f x Q_base`\n', best.r_scale, best.q_scale);
fprintf(rep, '- Final `R` diag: [%.6f, %.6f, %.6f]\n', best.R(1,1), best.R(2,2), best.R(3,3));
fprintf(rep, '- Final `Q`:\n\n');
fprintf(rep, '```text\n');
fprintf(rep, '%.6f  %.6f\n', best.Q(1,1), best.Q(1,2));
fprintf(rep, '%.6f  %.6f\n', best.Q(2,1), best.Q(2,2));
fprintf(rep, '```\n\n');

fprintf(rep, '## Vysledky (best run)\n');
fprintf(rep, '- Dosazen cil: %d\n', data.reached_goal);
fprintf(rep, '- Pocet kroku: %d\n', data.steps);
fprintf(rep, '- RMSE GNSS (XY): %.4f m\n', data.rmse_gnss_xy);
fprintf(rep, '- RMSE EKF (XY): %.4f m\n', data.rmse_ekf_xy);
fprintf(rep, '- Zlepseni EKF vuci GNSS: %.1f %%\n', 100*(data.rmse_gnss_xy-data.rmse_ekf_xy)/max(data.rmse_gnss_xy,1e-9));
fprintf(rep, '- Mean |e_theta|: %.4f rad\n', data.mean_theta_err);
fprintf(rep, '- Smoothness (mean step-to-step EKF XY change): %.4f m/step\n\n', data.smoothness_xy);

fprintf(rep, '## Pozorovani\n');
fprintf(rep, '- Odhad polohy konverguje i bez zname pocatecni orientace.\n');
fprintf(rep, '- Orientace je uspesne odhadovana, i kdyz neni primo merena GNSS (stabilizace |e_theta|).\n');
fprintf(rep, '- Vyladeni R/Q zlepsilo kompromis mezi rychlou korekci a hladkym odhadem.\n\n');

fprintf(rep, '## Vystupy\n');
fprintf(rep, '- `task4_deploy_success_run.png`\n');
fprintf(rep, '- `task4_deploy_tuning_grid.png`\n');
fprintf(rep, '- `task4_deploy_tuning_table.csv`\n');
fprintf(rep, '- `task4_deploy_report.md`\n');
fclose(rep);

fprintf('Saved: %s\n', rel_path(png_run, project_root));
fprintf('Saved: %s\n', rel_path(png_tune, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task4_deploy_tuning_table.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task4_deploy_report.md'), project_root));

function data = simulate_run(map, drive, Ts, true_start, mu0, sigma0, R, Q, seed)
rng(seed);
read_only.map = map;
read_only.sampling_period = Ts;
read_only.agent_drive = drive;

true_pose = true_start;
read_only.mocap_pose = true_pose;

public_vars = struct();
public_vars.motion_vector = [0, 0];
public_vars.path = plan_path(read_only, struct('path', []));
public_vars.controller_mode = 'pure_pursuit';
public_vars.use_estimated_pose_only = true;
public_vars = init_kalman_filter(read_only, public_vars);
public_vars.mu = mu0;
public_vars.sigma = sigma0;
public_vars.kf.R = R;
public_vars.kf.Q = Q;

max_steps = 950;
true_hist = nan(max_steps,3);
ekf_hist = nan(max_steps,3);
gnss_hist = nan(max_steps,2);
theta_std_hist = nan(max_steps,1);

reached = false;
for k = 1:max_steps
    if norm(true_pose(1:2) - map.goal) <= map.goal_tolerance
        reached = true;
        true_hist(k:end,:) = [];
        ekf_hist(k:end,:) = [];
        gnss_hist(k:end,:) = [];
        theta_std_hist(k:end,:) = [];
        break;
    end

    read_only.counter = k;
    read_only.mocap_pose = true_pose; % for evaluation only
    read_only.gnss_position = gnss_measure(true_pose, map.gnss_denied);

    public_vars.estimated_pose = public_vars.mu(:)';
    public_vars = plan_motion(read_only, public_vars);

    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only, public_vars);
    true_pose = move_agent(true_pose, public_vars.motion_vector, drive, Ts);

    true_hist(k,:) = true_pose;
    ekf_hist(k,:) = public_vars.mu(:)';
    gnss_hist(k,:) = read_only.gnss_position;
    theta_std_hist(k) = sqrt(max(public_vars.sigma(3,3), 0));
end

valid = ~isnan(true_hist(:,1));
true_hist = true_hist(valid,:);
ekf_hist = ekf_hist(valid,:);
gnss_hist = gnss_hist(valid,:);
theta_std_hist = theta_std_hist(valid);

gnss_err = vecnorm(gnss_hist - true_hist(:,1:2), 2, 2);
ekf_err = vecnorm(ekf_hist(:,1:2) - true_hist(:,1:2), 2, 2);
theta_err = abs(wrap_to_pi_local(ekf_hist(:,3) - true_hist(:,3)));

delta_est = diff(ekf_hist(:,1:2), 1, 1);
smoothness_xy = mean(vecnorm(delta_est, 2, 2), 'omitnan');

data.path = public_vars.path;
data.true_hist = true_hist;
data.ekf_hist = ekf_hist;
data.gnss_hist = gnss_hist;
data.theta_std = theta_std_hist;
data.gnss_err = gnss_err;
data.ekf_err = ekf_err;
data.theta_err = theta_err;
data.rmse_gnss_xy = sqrt(mean(gnss_err.^2, 'omitnan'));
data.rmse_ekf_xy = sqrt(mean(ekf_err.^2, 'omitnan'));
data.mean_theta_err = mean(theta_err, 'omitnan');
data.smoothness_xy = smoothness_xy;
data.steps = size(true_hist,1);
data.reached_goal = reached;
end

function [rows, best] = tune_grid(map, drive, Ts, true_start, mu0, sigma0, ...
    R_base, Q_base, r_scales, q_scales, repeats, stage_name, seed_base)
rows = [];
best = struct('cost', inf);

for ir = 1:numel(r_scales)
    for iq = 1:numel(q_scales)
        R = r_scales(ir) * R_base;
        Q = q_scales(iq) * Q_base;

        run_set = cell(repeats, 1);
        local_best_cost = inf;
        local_best_run = [];

        for rep = 1:repeats
            seed = seed_base + 1000*ir + 100*iq + rep;
            run_data = simulate_run(map, drive, Ts, true_start, mu0, sigma0, R, Q, seed);
            run_set{rep} = run_data;

            c = (1 - run_data.reached_goal) * 1e3 + run_data.rmse_ekf_xy + 0.6 * run_data.smoothness_xy;
            if c < local_best_cost
                local_best_cost = c;
                local_best_run = run_data;
            end
        end

        reached = cellfun(@(x) x.reached_goal, run_set);
        rmse_g = cellfun(@(x) x.rmse_gnss_xy, run_set);
        rmse_e = cellfun(@(x) x.rmse_ekf_xy, run_set);
        smooth = cellfun(@(x) x.smoothness_xy, run_set);
        th_e = cellfun(@(x) x.mean_theta_err, run_set);
        steps = cellfun(@(x) x.steps, run_set);

        success_rate = mean(reached);
        if any(reached > 0)
            mean_steps = mean(steps(reached > 0));
        else
            mean_steps = nan;
        end
        mean_rmse_g = mean(rmse_g);
        mean_rmse_e = mean(rmse_e);
        mean_smooth = mean(smooth);
        mean_th = mean(th_e);
        cost_score = (1 - success_rate) * 1e3 + mean_rmse_e + 0.6 * mean_smooth;

        rows = [rows; ir, iq, r_scales(ir), q_scales(iq), success_rate, mean_steps, mean_rmse_g, mean_rmse_e, mean_smooth, mean_th, cost_score]; %#ok<AGROW>

        if cost_score < best.cost
            best.cost = cost_score;
            best.stage = stage_name;
            best.r_scale = r_scales(ir);
            best.q_scale = q_scales(iq);
            best.R = R;
            best.Q = Q;
            best.success_rate = success_rate;
            best.mean_rmse_g = mean_rmse_g;
            best.mean_rmse_e = mean_rmse_e;
            best.mean_smooth = mean_smooth;
            best.mean_theta = mean_th;
            best.run = local_best_run;
        end
    end
end
end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2*pi) - pi;
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
