%% Task 4: Normal Distribution
% Build and plot normal pdf for LiDAR ch1 and GNSS X using sigma from Task 2.

clear; clc; close all;

task4_dir = fileparts(mfilename('fullpath'));               % .../Task2/Task4
task2_dir = fullfile(fileparts(task4_dir), 'Task2');        % .../Task2/Task2

if ~isfolder(task2_dir)
    error('Task2 folder not found: %s', task2_dir);
end

% LiDAR sigma from indoor_1 (channel 1)
lidar_file = fullfile(task2_dir, 'sensor_uncertainty_data_indoor_1.mat');
if ~isfile(lidar_file)
    error('Missing file: %s', lidar_file);
end
lidar_s = load(lidar_file, 'lidar_std');
sigma_lidar_ch1 = lidar_s.lidar_std(1);

% GNSS sigma from first file with valid X-axis std
files = dir(fullfile(task2_dir, 'sensor_uncertainty_data_*.mat'));
sigma_gnss_x = NaN;
gnss_source = '';
for k = 1:numel(files)
    f = fullfile(files(k).folder, files(k).name);
    s = load(f, 'gnss_std');
    if isfield(s, 'gnss_std') && numel(s.gnss_std) >= 1 && isfinite(s.gnss_std(1)) && s.gnss_std(1) > 0
        sigma_gnss_x = s.gnss_std(1);
        gnss_source = files(k).name;
        break;
    end
end

if ~isfinite(sigma_lidar_ch1) || sigma_lidar_ch1 <= 0
    error('Invalid LiDAR sigma for channel 1.');
end
if ~isfinite(sigma_gnss_x) || sigma_gnss_x <= 0
    error('No valid GNSS sigma_x found in Task2 data files.');
end

mu = 0;
x_limit = 4 * max(sigma_lidar_ch1, sigma_gnss_x);
x = linspace(-x_limit, x_limit, 1000);

pdf_lidar = norm_pdf(x, mu, sigma_lidar_ch1);
pdf_gnss = norm_pdf(x, mu, sigma_gnss_x);

figure('Name', 'Task 4 - Normal Distribution', 'NumberTitle', 'off');
plot(x, pdf_lidar, 'b-', 'LineWidth', 2); hold on;
plot(x, pdf_gnss, 'r-', 'LineWidth', 2);
grid on;
xlabel('x');
ylabel('pdf(x)');
title('Normal PDF of Sensor Noise (mu = 0)');
legend( ...
    sprintf('LiDAR ch1: sigma=%.4f', sigma_lidar_ch1), ...
    sprintf('GNSS X: sigma=%.4f', sigma_gnss_x), ...
    'Location', 'northeast');

fig_file = fullfile(task4_dir, 'task4_normal_distribution.fig');
png_file = fullfile(task4_dir, 'task4_normal_distribution.png');
savefig(fig_file);
saveas(gcf, png_file);

% Print workspace-relative paths.
project_root = task4_dir;
while ~exist(fullfile(project_root, 'main.m'), 'file')
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        project_root = '';
        break;
    end
    project_root = parent;
end

if ~isempty(project_root)
    prefix = [project_root filesep];
    task2_print = strrep(task2_dir, prefix, '');
    fig_print = strrep(fig_file, prefix, '');
    png_print = strrep(png_file, prefix, '');
else
    task2_print = task2_dir;
    fig_print = fig_file;
    png_print = png_file;
end

fprintf('========== TASK 4: NORMAL DISTRIBUTION ==========\n');
fprintf('Task2 data folder: %s\n', task2_print);
fprintf('mu = %.1f\n', mu);
fprintf('sigma LiDAR ch1 = %.6f (source: sensor_uncertainty_data_indoor_1.mat)\n', sigma_lidar_ch1);
fprintf('sigma GNSS X    = %.6f (source: %s)\n', sigma_gnss_x, gnss_source);
fprintf('Saved figure: %s\n', fig_print);
fprintf('Saved figure: %s\n', png_print);
