%% Task5/Task2 - EKF prediction/correction validation
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
rng(22);

%% Setup environment
map = load_map(fullfile(project_root, 'maps', 'outdoor_1.txt'));
map.goal_tolerance = 0.5;
Ts = 0.1;
drive = struct('type', 2, 'interwheel_dist', 0.2, 'max_vel', 1.0);

start_pose = [2, 2, pi/2];
true_pose = start_pose;

read_only.map = map;
read_only.sampling_period = Ts;
read_only.agent_drive = drive;
read_only.mocap_pose = true_pose;

public_vars = struct();
public_vars.motion_vector = [0, 0];
public_vars.path = plan_path(read_only, struct('path', []));
public_vars.controller_mode = 'pure_pursuit';

% Initialize KF and override by known start for Task5/Task2 testing.
public_vars = init_kalman_filter(read_only, public_vars);
public_vars.mu = start_pose(:);
public_vars.sigma = zeros(3, 3);

% If available, use GNSS covariance estimated in Task5/Task1.
gnss_init_file = fullfile(project_root, 'algorithms', 'tasks', 'Task5', 'Task1', 'task1_gnss_init_data.mat');
if isfile(gnss_init_file)
    s = load(gnss_init_file, 'gnss_sigma');
    if isfield(s, 'gnss_sigma') && all(size(s.gnss_sigma) == [2, 2])
        public_vars.kf.Q = s.gnss_sigma;
    end
end

max_steps = 500;
true_hist = nan(max_steps, 3);
gnss_hist = nan(max_steps, 2);
ekf_hist = nan(max_steps, 3);

for k = 1:max_steps
    if norm(true_pose(1:2) - map.goal) <= map.goal_tolerance
        true_hist(k:end, :) = [];
        gnss_hist(k:end, :) = [];
        ekf_hist(k:end, :) = [];
        break;
    end

    read_only.counter = k;
    read_only.mocap_pose = true_pose;
    read_only.gnss_position = gnss_measure(true_pose, map.gnss_denied);

    % Control (for fair trajectory following in this validation, use mocap).
    public_vars.estimated_pose = public_vars.mu(:)';
    public_vars = plan_motion(read_only, public_vars);

    % EKF update with current u,z.
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only, public_vars);

    % True motion propagation.
    true_pose = move_agent(true_pose, public_vars.motion_vector, drive, Ts);

    true_hist(k, :) = true_pose;
    gnss_hist(k, :) = read_only.gnss_position;
    ekf_hist(k, :) = public_vars.mu(:)';
end

valid = ~isnan(true_hist(:, 1));
true_hist = true_hist(valid, :);
gnss_hist = gnss_hist(valid, :);
ekf_hist = ekf_hist(valid, :);

gnss_err = vecnorm(gnss_hist - true_hist(:, 1:2), 2, 2);
ekf_err = vecnorm(ekf_hist(:, 1:2) - true_hist(:, 1:2), 2, 2);
theta_err = abs(wrap_to_pi_local(ekf_hist(:, 3) - true_hist(:, 3)));

rmse_gnss = sqrt(mean(gnss_err.^2, 'omitnan'));
rmse_ekf = sqrt(mean(ekf_err.^2, 'omitnan'));

%% Figure
fig = figure('Name', 'Task5/Task2 EKF validation', 'NumberTitle', 'off', 'Color', 'w', 'Position', [100 100 1200 620]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile([2 1]);
hold on; axis equal; grid on;
for i = 1:size(map.walls, 1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 3, 'HandleVisibility', 'off');
end
plot(public_vars.path(:,1), public_vars.path(:,2), '-', 'Color', [0.15 0.7 0.25], 'LineWidth', 2.2, 'DisplayName', 'Reference path');
plot(true_hist(:,1), true_hist(:,2), '-', 'Color', [0.1 0.3 0.95], 'LineWidth', 1.8, 'DisplayName', 'True trajectory');
plot(ekf_hist(:,1), ekf_hist(:,2), '--', 'Color', [0.9 0.2 0.2], 'LineWidth', 1.8, 'DisplayName', 'EKF estimate');
scatter(gnss_hist(:,1), gnss_hist(:,2), 10, [0.4 0.4 0.4], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'GNSS samples');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');
title('Outdoor\_1 trajectory: true vs EKF');
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5 map.limits(3)+0.5 map.limits(2)-0.5 map.limits(4)+0.5]);

nexttile;
plot(gnss_err, 'Color', [0.45 0.45 0.45], 'LineWidth', 1.4, 'DisplayName', 'GNSS XY error'); hold on;
plot(ekf_err, 'Color', [0.85 0.2 0.2], 'LineWidth', 1.8, 'DisplayName', 'EKF XY error');
grid on;
xlabel('Step'); ylabel('Error [m]');
title(sprintf('XY error (RMSE GNSS=%.3f, EKF=%.3f)', rmse_gnss, rmse_ekf));
legend('Location', 'best');

nexttile;
plot(theta_err, 'Color', [0.55 0.25 0.8], 'LineWidth', 1.6);
grid on;
xlabel('Step'); ylabel('|e_\\theta| [rad]');
title('EKF heading error');

png_path = fullfile(out_dir, 'task2_ekf_validation.png');
fig_path = fullfile(out_dir, 'task2_ekf_validation.fig');
savefig(fig, fig_path);
exportgraphics(fig, png_path, 'BackgroundColor', 'white', 'Resolution', 160);

summary_tbl = table((1:numel(ekf_err))', gnss_err, ekf_err, theta_err, ...
    'VariableNames', {'step', 'gnss_xy_error', 'ekf_xy_error', 'ekf_theta_error'});
writetable(summary_tbl, fullfile(out_dir, 'task2_ekf_validation_errors.csv'));

rep = fopen(fullfile(out_dir, 'task2_ekf_validation_report.md'), 'w');
fprintf(rep, '# Task5/Task2 - EKF validace (predikce + korekce)\n\n');
fprintf(rep, '- Mapa: `outdoor_1`\n');
fprintf(rep, '- Start: `[2,2,pi/2]`\n');
fprintf(rep, '- Pocet kroku: %d\n\n', numel(ekf_err));
fprintf(rep, '## Souhrn metrik\n');
fprintf(rep, '- RMSE GNSS (XY): %.4f m\n', rmse_gnss);
fprintf(rep, '- RMSE EKF (XY): %.4f m\n', rmse_ekf);
fprintf(rep, '- Zlepseni EKF vuci GNSS: %.1f %%\n', 100 * (rmse_gnss - rmse_ekf) / max(rmse_gnss, 1e-9));
fprintf(rep, '- Mean |e_theta|: %.4f rad\n', mean(theta_err));
fprintf(rep, '- Max  |e_theta|: %.4f rad\n\n', max(theta_err));
fprintf(rep, '## Poznamka\n');
fprintf(rep, '- V tomto testu je rideni vedeno po referencni trajektorii a EKF je validovan proti true poloze.\n');
fclose(rep);

fprintf('Saved: %s\n', rel_path(png_path, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task2_ekf_validation_report.md'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task2_ekf_validation_errors.csv'), project_root));

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2 * pi) - pi;
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
