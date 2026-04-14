%% Task4/Task3 - Compare resampling algorithms
clear; clc; close all;

project_root = fileparts(mfilename('fullpath'));
while ~exist(fullfile(project_root, 'main.m'), 'file')
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        error('Could not locate project root (main.m).');
    end
    project_root = parent;
end

addpath(fullfile(project_root, 'algorithms'));
addpath(fullfile(project_root, 'algorithms', 'particle_filter'));

out_dir = fileparts(mfilename('fullpath'));

methods = {'multinomial', 'systematic', 'stratified', 'residual'};
M = numel(methods);

N = 1200;        % particles per trial
trials = 250;    % Monte Carlo trials

mean_err = zeros(trials, M);
unique_ratio = zeros(trials, M);
runtime_ms = zeros(trials, M);

rng(123);

for t = 1:trials
    % Synthetic cloud + weighted likelihood around reference pose.
    particles = [10 * rand(N, 1), 10 * rand(N, 1), -pi + 2 * pi * rand(N, 1)];
    ref_pose = [2 + 6 * rand(), 2 + 6 * rand(), -pi + 2 * pi * rand()];

    dxy2 = (particles(:, 1) - ref_pose(1)).^2 + (particles(:, 2) - ref_pose(2)).^2;
    dth = wrap_to_pi_local(particles(:, 3) - ref_pose(3));
    w = exp(-dxy2 / (2 * 0.7^2) - (dth.^2) / (2 * 0.4^2));
    w = w / sum(w);

    weighted_mean = [sum(w .* particles(:, 1)), sum(w .* particles(:, 2)), circ_mean(particles(:, 3), w)];

    for m = 1:M
        tic;
        p_new = resample_particles(particles, w, methods{m});
        runtime_ms(t, m) = 1000 * toc;

        sample_mean = [mean(p_new(:, 1)), mean(p_new(:, 2)), circ_mean_uniform(p_new(:, 3))];
        mean_err(t, m) = norm(sample_mean(1:2) - weighted_mean(1:2));

        unique_ratio(t, m) = size(unique(p_new, 'rows'), 1) / N;
    end
end

summary = table(methods', ...
    mean(mean_err, 1)', std(mean_err, 0, 1)', ...
    mean(unique_ratio, 1)', std(unique_ratio, 0, 1)', ...
    mean(runtime_ms, 1)', ...
    'VariableNames', {'method', 'mean_xy_error_m', 'std_xy_error_m', 'mean_unique_ratio', 'std_unique_ratio', 'mean_runtime_ms'});

writetable(summary, fullfile(out_dir, 'task3_resampling_comparison.csv'));

fig = figure('Name', 'Task4/Task3 resampling comparison', 'NumberTitle', 'off', 'Color', 'w', ...
    'Position', [100 100 1280 560]);
t = tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

method_names = string(summary.method);
x = 1:height(summary);
method_colors = [
    0.30 0.45 0.85;   % multinomial
    0.12 0.62 0.38;   % systematic
    0.87 0.49 0.14;   % stratified
    0.60 0.36 0.74    % residual
];

nexttile;
b1 = bar(x, summary.mean_xy_error_m, 0.7, 'EdgeColor', 'none');
b1.CData = method_colors;
hold on;
errorbar(x, summary.mean_xy_error_m, summary.std_xy_error_m, 'k.', 'LineWidth', 1);
hold off;
style_axes(gca, x, method_names);
ylabel('Mean XY error [m]');
title('Accuracy (lower is better)');
annotate_bars(x, summary.mean_xy_error_m, '%.4f');

nexttile;
b2 = bar(x, summary.mean_unique_ratio, 0.7, 'EdgeColor', 'none');
b2.CData = method_colors;
hold on;
errorbar(x, summary.mean_unique_ratio, summary.std_unique_ratio, 'k.', 'LineWidth', 1);
hold off;
style_axes(gca, x, method_names);
ylabel('Unique particle ratio [-]');
title('Diversity (higher is better)');
annotate_bars(x, summary.mean_unique_ratio, '%.3f');

nexttile;
b3 = bar(x, summary.mean_runtime_ms, 0.7, 'EdgeColor', 'none');
b3.CData = method_colors;
style_axes(gca, x, method_names);
ylabel('Runtime per resampling [ms]');
title('Speed (lower is better)');
annotate_bars(x, summary.mean_runtime_ms, '%.4f');

title(t, sprintf('Task4/Task3: comparison over %d trials (N=%d)', trials, N), ...
    'FontWeight', 'bold', 'FontSize', 15);

png_path = fullfile(out_dir, 'task3_resampling_comparison.png');
fig_path = fullfile(out_dir, 'task3_resampling_comparison.fig');
savefig(fig, fig_path);
exportgraphics(fig, png_path, 'BackgroundColor', 'white', 'Resolution', 150);

md = fopen(fullfile(out_dir, 'task3_resampling_report.md'), 'w');
fprintf(md, '# Task4/Task3 - Porovnani resamplingu\n\n');
fprintf(md, '- Pocet pokusu: %d\n', trials);
fprintf(md, '- Pocet castic: %d\n\n', N);
fprintf(md, '| Metoda | Mean XY error [m] | Std XY error [m] | Unique ratio | Runtime [ms] |\n');
fprintf(md, '|---|---:|---:|---:|---:|\n');
for i = 1:height(summary)
    fprintf(md, '| %s | %.4f | %.4f | %.3f | %.4f |\n', summary.method{i}, ...
        summary.mean_xy_error_m(i), summary.std_xy_error_m(i), summary.mean_unique_ratio(i), summary.mean_runtime_ms(i));
end
fprintf(md, '\nDoporuceni: pro tento projekt je vychozi volba `systematic` kvuli nizke varianci a dobre rychlosti.\n');
fclose(md);

fprintf('Saved: %s\n', rel_path(png_path, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task3_resampling_comparison.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task3_resampling_report.md'), project_root));

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2 * pi) - pi;
end

function m = circ_mean(angles, weights)
m = atan2(sum(weights .* sin(angles)), sum(weights .* cos(angles)));
end

function m = circ_mean_uniform(angles)
m = atan2(mean(sin(angles)), mean(cos(angles)));
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end

function style_axes(ax, x, method_names)
ax.FontName = 'Segoe UI';
ax.FontSize = 11;
ax.Box = 'on';
ax.LineWidth = 0.8;
ax.XGrid = 'off';
ax.YGrid = 'on';
ax.GridAlpha = 0.15;
ax.GridColor = [0 0 0];
ax.XTick = x;
ax.XTickLabel = method_names;
ax.XTickLabelRotation = 20;
end

function annotate_bars(x, y, fmt)
dy = max(y) * 0.03 + 1e-6;
for i = 1:numel(x)
    text(x(i), y(i) + dy, sprintf(fmt, y(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 9, 'Color', [0.2 0.2 0.2], 'FontName', 'Segoe UI');
end
end
