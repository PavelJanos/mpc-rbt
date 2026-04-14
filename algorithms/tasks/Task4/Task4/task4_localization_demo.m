%% Task4/Task4 - Particle filter localization demo
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
addpath(fullfile(project_root, 'algorithms', 'particle_filter'));

out_dir = fileparts(mfilename('fullpath'));
rng(11);

%% Environment
map = load_map(fullfile(project_root, 'maps', 'indoor_1.txt'));
map.goal_tolerance = 0.5;
lidar_cfg = [0, 45, 90, 135, 180, 225, 270, 315] / 180 * pi;

read_only_vars.map = map;
read_only_vars.agent_drive = struct('type', 2, 'interwheel_dist', 0.2, 'max_vel', 1);
read_only_vars.sampling_period = 0.1;
read_only_vars.lidar_config = lidar_cfg;

public_vars.motion_vector = [0, 0];
public_vars.path = [];
public_vars.estimated_pose = [];
public_vars.particles = [];
public_vars.controller_mode = 'pure_pursuit';

% Start at Task3 path start.
true_pose = [2, 8.5, pi/2];
public_vars.path = plan_path(read_only_vars, public_vars);
public_vars = init_particle_filter(read_only_vars, public_vars);

max_steps = 420;
true_hist = nan(max_steps, 3);
est_hist = nan(max_steps, 3);
cluster_hist = nan(max_steps, 1);

best_idx = 1;
best_cluster = inf;

for k = 1:max_steps
    if norm(true_pose(1:2) - map.goal) <= map.goal_tolerance
        true_hist(k:end, :) = [];
        est_hist(k:end, :) = [];
        cluster_hist(k:end) = [];
        break;
    end

    read_only_vars.counter = k;
    read_only_vars.mocap_pose = true_pose;
    [read_only_vars.lidar_distances, ~] = lidar_measure(map, true_pose, lidar_cfg);

    % PF update: prediction + correction + resampling
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    public_vars.estimated_pose = estimate_pose(public_vars);

    % Path tracking command for next move (controller uses mocap here).
    public_vars.path = plan_path(read_only_vars, public_vars);
    public_vars = plan_motion(read_only_vars, public_vars);

    % Move true robot.
    true_pose = move_agent(true_pose, public_vars.motion_vector, read_only_vars.agent_drive, read_only_vars.sampling_period);

    true_hist(k, :) = true_pose;
    est_hist(k, :) = public_vars.estimated_pose;
    cluster_hist(k) = median(vecnorm(public_vars.particles(:, 1:2) - true_pose(1:2), 2, 2));

    if k > 20 && cluster_hist(k) < best_cluster
        best_cluster = cluster_hist(k);
        best_idx = k;
        best_particles = public_vars.particles;
        best_true_pose = true_pose;
        best_est_pose = public_vars.estimated_pose;
    end
end

valid = ~isnan(cluster_hist);
true_hist = true_hist(valid, :);
est_hist = est_hist(valid, :);
cluster_hist = cluster_hist(valid);

if ~exist('best_particles', 'var')
    best_particles = public_vars.particles;
    best_true_pose = true_pose;
    best_est_pose = public_vars.estimated_pose;
    best_idx = numel(cluster_hist);
    best_cluster = cluster_hist(end);
end

%% Figure 1: best convergence snapshot
fig1 = figure('Name', 'Task4 Task4 - PF cluster snapshot', 'NumberTitle', 'off', 'Color', 'w', 'Position', [120 120 950 760]);
hold on; axis equal; grid on;
for i = 1:size(map.walls, 1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], 'Color', 'k', 'LineWidth', 4, 'HandleVisibility', 'off');
end
plot(public_vars.path(:,1), public_vars.path(:,2), 'Color', [0.15 0.7 0.2], 'LineWidth', 2.2, 'DisplayName', 'Reference path');
plot(true_hist(:,1), true_hist(:,2), '-', 'Color', [0.2 0.45 0.95], 'LineWidth', 1.8, 'DisplayName', 'True trajectory');
scatter(best_particles(:,1), best_particles(:,2), 22, ...
    'Marker', 's', ...
    'MarkerFaceColor', [0.05 0.75 0.85], ...
    'MarkerEdgeColor', [0.00 0.35 0.45], ...
    'LineWidth', 0.35, ...
    'MarkerFaceAlpha', 0.75, ...
    'MarkerEdgeAlpha', 0.95, ...
    'DisplayName', 'Particles');
plot(best_true_pose(1), best_true_pose(2), 'ko', 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', 'True pose');
plot(best_est_pose(1), best_est_pose(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'PF estimate');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');
title(sprintf('PF convergence snapshot (step %d, median cluster radius %.3f m)', best_idx, best_cluster));
xlabel('x [m]');
ylabel('y [m]');
legend('Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5 map.limits(3)+0.5 map.limits(2)-0.5 map.limits(4)+0.5]);

snap_png = fullfile(out_dir, 'task4_pf_cluster_snapshot.png');
snap_fig = fullfile(out_dir, 'task4_pf_cluster_snapshot.fig');
savefig(fig1, snap_fig);
exportgraphics(fig1, snap_png, 'BackgroundColor', 'white', 'Resolution', 160);

%% Figure 2: localization error and cluster over time
xy_err = vecnorm(est_hist(:,1:2) - true_hist(:,1:2), 2, 2);
fig2 = figure('Name', 'Task4 Task4 - PF metrics', 'NumberTitle', 'off', 'Color', 'w', 'Position', [130 130 960 420]);
tiledlayout(1,2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(xy_err, 'LineWidth', 1.8);
grid on;
xlabel('Step');
ylabel('||est - true||_{xy} [m]');
title('Localization XY error');

nexttile;
plot(cluster_hist, 'LineWidth', 1.8, 'Color', [0.82 0.33 0.1]);
grid on;
xlabel('Step');
ylabel('Median particle distance to true pose [m]');
title('Particle cluster compactness');

metrics_png = fullfile(out_dir, 'task4_pf_metrics.png');
metrics_fig = fullfile(out_dir, 'task4_pf_metrics.fig');
savefig(fig2, metrics_fig);
exportgraphics(fig2, metrics_png, 'BackgroundColor', 'white', 'Resolution', 160);

%% Report
rep = fopen(fullfile(out_dir, 'task4_task4_report.md'), 'w');
fprintf(rep, '# Task4/Task4 - Lokalizace partičním filtrem\n\n');
fprintf(rep, '## Nastaveni\n');
fprintf(rep, '- Mapa: `indoor_1`\n');
fprintf(rep, '- Start: `(2, 8.5, pi/2)`\n');
fprintf(rep, '- Pocet castic: %d\n', size(best_particles, 1));
fprintf(rep, '- Predikce: probabilisticky motion model (`predict_pose`)\n');
fprintf(rep, '- Korekce: LiDAR model + gaussovske vahovani (`weight_particles`)\n');
fprintf(rep, '- Resampling: `systematic`\n\n');

fprintf(rep, '## Vysledky\n');
fprintf(rep, '- Nejlepsi detekovana kompaktnost klastru (median vzdalenosti): %.3f m\n', best_cluster);
fprintf(rep, '- Prumerna XY chyba odhadu: %.3f m\n', mean(xy_err, 'omitnan'));
fprintf(rep, '- Minimalni XY chyba odhadu: %.3f m\n', min(xy_err));
fprintf(rep, '- Maximalni XY chyba odhadu: %.3f m\n\n', max(xy_err));

fprintf(rep, '## Diskuze nejdulezitejsich parametru\n');
fprintf(rep, '- Pocet castic (`N`): vyssi N zvysuje robustnost a presnost, ale roste vypocetni narocnost.\n');
fprintf(rep, '- `sigma_lidar` ve vahovani: mala hodnota vede k ostremu vahovani a riziku degenerace, velka hodnota zhorsuje rozliseni mezi casticemi.\n');
fprintf(rep, '- Sila sumu v `predict_pose`: prilis maly sum omezuje prohledani stavu, prilis velky sum zhorsuje stabilitu odhadu.\n');
fprintf(rep, '- Volba resamplingu: `systematic` poskytuje nizkou varianci a dobry kompromis rychlost/kvalita.\n\n');

fprintf(rep, '## Hlavni problem a jeho reseni\n');
fprintf(rep, '- Hlavni problem byla degenerace vah pri vetsim nesouladu mereni. Resenim bylo stabilni gaussovske vahovani, validace kanalu LiDARu a systematic resampling.\n\n');

fprintf(rep, '## Vystupy\n');
fprintf(rep, '- `task4_pf_cluster_snapshot.png`\n');
fprintf(rep, '- `task4_pf_metrics.png`\n');
fprintf(rep, '- `task4_task4_report.md`\n');
fclose(rep);

fprintf('Saved: %s\n', rel_path(snap_png, project_root));
fprintf('Saved: %s\n', rel_path(metrics_png, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task4_task4_report.md'), project_root));

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
