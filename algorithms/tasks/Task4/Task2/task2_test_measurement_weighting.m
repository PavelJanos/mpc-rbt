%% Task4/Task2 - Test compute_lidar_measurement and weight_particles
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
addpath(fullfile(project_root, 'algorithms', 'particle_filter'));

out_dir = fileparts(mfilename('fullpath'));

map = load_map(fullfile(project_root, 'maps', 'indoor_1.txt'));
lidar_cfg = [0, 45, 90, 135, 180, 225, 270, 315] / 180 * pi;
pose_ref = [2, 8.5, pi/2];

%% A) Measurement function test
z_model = compute_lidar_measurement(map, pose_ref, lidar_cfg);
[z_sim, ~] = lidar_measure(map, pose_ref, lidar_cfg);
abs_diff = abs(z_model - z_sim);

%% B) Weighting monotonicity test
rng(7);
N = 600;
noise_levels = [0.00, 0.02, 0.05, 0.10, 0.20, 0.35];
avg_weight = zeros(size(noise_levels));
std_weight = zeros(size(noise_levels));
max_weight = zeros(size(noise_levels));
ess_vals = zeros(size(noise_levels));

for i = 1:numel(noise_levels)
    sigma = noise_levels(i);
    particles_meas = repmat(z_model, N, 1) + sigma * randn(N, numel(z_model));
    w = weight_particles(particles_meas, z_model);
    avg_weight(i) = mean(w);
    std_weight(i) = std(w);
    max_weight(i) = max(w);
    ess_vals(i) = 1 / sum(w.^2);
end

% Ranking sanity check (ideal vs small/large noise)
pm_rank = [
    z_model;
    z_model + 0.03 * randn(1, numel(z_model));
    z_model + 0.25 * randn(1, numel(z_model))
];
w_rank = weight_particles(pm_rank, z_model);
ranking_ok = (w_rank(1) > w_rank(2)) && (w_rank(2) > w_rank(3));
sum_ok = abs(sum(w_rank) - 1) < 1e-9;

% NaN/Inf robustness
z_corrupt = z_model;
z_corrupt(2) = inf;
z_corrupt(5) = nan;
w_corrupt = weight_particles(pm_rank, z_corrupt);
corrupt_sum_ok = abs(sum(w_corrupt) - 1) < 1e-9;

%% Plot
fig = figure('Name', 'Task4/Task2 test', 'NumberTitle', 'off', 'Color', 'w', 'Position', [120 120 1100 500]);
t = tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
bar(abs_diff, 'FaceColor', [0.2 0.5 0.9]);
grid on;
xlabel('LiDAR channel');
ylabel('|model - simulator| [m]');
title('Measurement consistency check');

nexttile;
yyaxis left;
plot(noise_levels, max_weight, 'o-', 'LineWidth', 1.8, 'MarkerSize', 6, 'DisplayName', 'Max(w)');
ylabel('Maximalni vaha');
ylim([0, max(max_weight) * 1.15 + eps]);

yyaxis right;
plot(noise_levels, ess_vals, 's-', 'LineWidth', 1.8, 'MarkerSize', 6, 'DisplayName', 'ESS');
ylabel('ESS = 1 / sum(w^2)');
ylim([0, N * 1.05]);

grid on;
xlabel('Injected measurement noise std [m]');
title('Vazeni castic vs nesoulad mereni');
legend('Location', 'best');

title(t, 'Task4/Task2: measurement and weighting tests');

fig_path = fullfile(out_dir, 'task2_measurement_weighting_test.png');
saveas(fig, fig_path);

%% Text report
report_path = fullfile(out_dir, 'task2_test_report.md');
f = fopen(report_path, 'w');
if f < 0
    error('Cannot write report file.');
end
fprintf(f, '# Task4/Task2 - Testovaci report\n\n');
fprintf(f, '## compute_lidar_measurement\n');
fprintf(f, '- Prumerna odchylka |model-simulator|: %.4f m\n', mean(abs_diff));
fprintf(f, '- Maximalni odchylka |model-simulator|: %.4f m\n', max(abs_diff));
fprintf(f, '- Poznamka: simulatorova funkce muze obsahovat mereni se sumem.\n\n');

fprintf(f, '## weight_particles\n');
fprintf(f, '- Test poradi vah (idealni > maly sum > velky sum): %s\n', tf2str(ranking_ok));
fprintf(f, '- Test normalizace sum(weights)=1: %s\n', tf2str(sum_ok));
fprintf(f, '- Robustnost pro NaN/Inf v mereni: %s\n', tf2str(corrupt_sum_ok));
fprintf(f, '- Vahy (test poradi): [%.4g, %.4g, %.4g]\n', w_rank(1), w_rank(2), w_rank(3));
fprintf(f, '- Pri sigma=0.00: max(w)=%.4g, ESS=%.1f\n', max_weight(1), ess_vals(1));
fprintf(f, '- Pri sigma=0.35: max(w)=%.4g, ESS=%.1f\n\n', max_weight(end), ess_vals(end));

fprintf(f, '## Soubory\n');
fprintf(f, '- task2_measurement_weighting_test.png\n');
fprintf(f, '- task2_test_report.md\n');
fclose(f);

fprintf('Saved: %s\n', rel_path(fig_path, project_root));
fprintf('Saved: %s\n', rel_path(report_path, project_root));

function s = tf2str(tf)
if tf
    s = 'PASS';
else
    s = 'FAIL';
end
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
