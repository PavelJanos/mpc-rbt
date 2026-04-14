%% Task5/Task1 - Outdoor path + GNSS initialization
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

%% Load outdoor_1 and get manual path from plan_path
map = load_map(fullfile(project_root, 'maps', 'outdoor_1.txt'));
read_only_vars.map = map;
public_vars.path = [];
path = plan_path(read_only_vars, public_vars);

%% GNSS uncertainty initialization at start pose x=[2,2,pi/2]
start_pose = [2, 2, pi/2];
N = 450;
gnss_data = nan(N, 2);

for i = 1:N
    gnss_data(i, :) = gnss_measure(start_pose, map.gnss_denied);
end

valid = ~any(isnan(gnss_data) | isinf(gnss_data), 2);
gnss_valid = gnss_data(valid, :);

gnss_mu = mean(gnss_valid, 1);
gnss_sigma = cov(gnss_valid);     % 2x2 covariance matrix
gnss_std = std(gnss_valid, 0, 1); % [std_x, std_y]

%% Save numeric outputs
save(fullfile(out_dir, 'task1_gnss_init_data.mat'), ...
    'start_pose', 'gnss_valid', 'gnss_mu', 'gnss_sigma', 'gnss_std', 'path');
writematrix(gnss_valid, fullfile(out_dir, 'task1_gnss_samples.csv'));
writematrix(gnss_sigma, fullfile(out_dir, 'task1_gnss_covariance.csv'));

%% Figure: path + GNSS samples with covariance ellipse
fig = figure('Name', 'Task5/Task1 preparation', 'NumberTitle', 'off', 'Color', 'w', 'Position', [110 110 1200 520]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on; axis equal; grid on;
for i = 1:size(map.walls, 1)
    line([map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], 'Color', [0.1 0.1 0.1], 'LineWidth', 3, 'HandleVisibility', 'off');
end
plot(path(:,1), path(:,2), '-', 'Color', [0.15 0.65 0.25], 'LineWidth', 2.2, 'DisplayName', 'Manual trajectory');
plot(start_pose(1), start_pose(2), 'bo', 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', 'Start [2,2]');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal [16,2]');
title('Outdoor\_1 manual trajectory');
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'eastoutside', 'Interpreter', 'none');
axis([map.limits(1)-0.5 map.limits(3)+0.5 map.limits(2)-0.5 map.limits(4)+0.5]);

nexttile;
hold on; axis equal; grid on;
scatter(gnss_valid(:,1), gnss_valid(:,2), 8, [0.2 0.45 0.9], 'filled', 'MarkerFaceAlpha', 0.25, 'DisplayName', 'GNSS samples');
plot(start_pose(1), start_pose(2), 'k+', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'True start');
plot(gnss_mu(1), gnss_mu(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Estimated mean \mu');
[ex, ey] = covariance_ellipse_2d(gnss_mu, gnss_sigma, 2.0, 200);
plot(ex, ey, 'r-', 'LineWidth', 1.8, 'DisplayName', '2\sigma ellipse');
title('GNSS initialization statistics at start');
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'best');

fig_png = fullfile(out_dir, 'task1_outdoor_path_gnss_init.png');
fig_fig = fullfile(out_dir, 'task1_outdoor_path_gnss_init.fig');
savefig(fig, fig_fig);
exportgraphics(fig, fig_png, 'BackgroundColor', 'white', 'Resolution', 160);

%% Short report (CZ)
f = fopen(fullfile(out_dir, 'task1_preparation_report.md'), 'w');
fprintf(f, '# Task5/Task1 - Příprava (outdoor_1)\n\n');
fprintf(f, '## Trajektorie\n');
fprintf(f, '- Mapa: `outdoor_1`\n');
fprintf(f, '- Start: `[2,2,pi/2]`\n');
fprintf(f, '- Cíl: `[16,2]`\n');
fprintf(f, '- Trajektorie je ručně definována v `plan_path.m` (obsahuje křivkové segmenty).\n\n');

fprintf(f, '## Inicializace GNSS (statické vzorkování v bodě startu)\n');
fprintf(f, '- Počet platných vzorků: %d\n', size(gnss_valid,1));
fprintf(f, '- Odhad středu `mu = [%.4f, %.4f]`\n', gnss_mu(1), gnss_mu(2));
fprintf(f, '- Odhad směrodatných odchylek `std = [%.4f, %.4f]`\n', gnss_std(1), gnss_std(2));
fprintf(f, '- Kovarianční matice:\n\n');
fprintf(f, '```text\n');
fprintf(f, '%.6f  %.6f\n', gnss_sigma(1,1), gnss_sigma(1,2));
fprintf(f, '%.6f  %.6f\n', gnss_sigma(2,1), gnss_sigma(2,2));
fprintf(f, '```\n\n');
fprintf(f, 'Tyto hodnoty (`mu`, `Sigma`) lze přímo použít jako inicializační odhad GNSS pro další EKF úkoly.\n');
fclose(f);

fprintf('Saved: %s\n', rel_path(fig_png, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task1_preparation_report.md'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task1_gnss_init_data.mat'), project_root));

function [x, y] = covariance_ellipse_2d(mu, Sigma, n_sigma, n_pts)
[V, D] = eig(Sigma);
t = linspace(0, 2*pi, n_pts);
circle = [cos(t); sin(t)];
ell = n_sigma * V * sqrt(D) * circle;
x = mu(1) + ell(1, :);
y = mu(2) + ell(2, :);
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
