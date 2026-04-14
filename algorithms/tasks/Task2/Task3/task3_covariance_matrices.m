%% Task 3: Covariance Matrices
% Builds covariance matrices from Task 2 sensor data and stores results
% in Task2/Task3 for each available map dataset.

clear; clc;

this_dir = fileparts(mfilename('fullpath'));                  % .../Task2/Task3
task2_data_dir = fullfile(fileparts(this_dir), 'Task2');      % .../Task2/Task2

% Resolve project root to print workspace-relative paths.
project_root = this_dir;
while ~exist(fullfile(project_root, 'main.m'), 'file')
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        project_root = '';
        break;
    end
    project_root = parent;
end

if ~isfolder(task2_data_dir)
    error('Task2 data folder not found: %s', task2_data_dir);
end

files = dir(fullfile(task2_data_dir, 'sensor_uncertainty_data_*.mat'));
if isempty(files)
    error('No Task2 data files found in %s', task2_data_dir);
end

fprintf('========== TASK 3: COVARIANCE MATRICES ==========\n');
if ~isempty(project_root)
    prefix = [project_root filesep];
    source_dir_print = strrep(task2_data_dir, prefix, '');
    output_dir_print = strrep(this_dir, prefix, '');
else
    source_dir_print = task2_data_dir;
    output_dir_print = this_dir;
end
fprintf('Source folder: %s\n', source_dir_print);
fprintf('Output folder: %s\n\n', output_dir_print);

for k = 1:numel(files)
    in_file = fullfile(files(k).folder, files(k).name);
    s = load(in_file, 'lidar_data', 'gnss_data', 'lidar_std', 'gnss_std');

    if ~isfield(s, 'lidar_data') || ~isfield(s, 'gnss_data')
        fprintf('[SKIP] %s (missing lidar_data/gnss_data)\n', files(k).name);
        continue;
    end

    lidar_data = s.lidar_data;
    gnss_data = s.gnss_data;

    % Remove invalid measurements before covariance computation
    lidar_valid = lidar_data(~any(isnan(lidar_data) | isinf(lidar_data), 2), :);
    gnss_valid = gnss_data(~any(isnan(gnss_data) | isinf(gnss_data), 2), :);

    Sigma_lidar = nan(8, 8);
    Sigma_gnss = nan(2, 2);
    lidar_std_filtered = nan(1, 8);
    gnss_std_filtered = nan(1, 2);
    lidar_diag_err = nan;
    gnss_diag_err = nan;

    has_lidar_cov = size(lidar_valid, 1) >= 2;
    has_gnss_cov = size(gnss_valid, 1) >= 2;

    if has_lidar_cov
        Sigma_lidar = cov(lidar_valid);    % expected 8x8
        lidar_std_filtered = std(lidar_valid);
        lidar_diag_err = max(abs(diag(Sigma_lidar)' - lidar_std_filtered.^2));
    end
    if has_gnss_cov
        Sigma_gnss = cov(gnss_valid);      % expected 2x2
        gnss_std_filtered = std(gnss_valid);
        gnss_diag_err = max(abs(diag(Sigma_gnss)' - gnss_std_filtered.^2));
    end

    [~, map_base, ~] = fileparts(in_file);
    map_base = strrep(map_base, 'sensor_uncertainty_data_', '');
    out_file = fullfile(this_dir, sprintf('covariance_data_%s.mat', map_base));

    save(out_file, ...
        'Sigma_lidar', 'Sigma_gnss', ...
        'lidar_std_filtered', 'gnss_std_filtered', ...
        'lidar_diag_err', 'gnss_diag_err', ...
        'in_file');

    fprintf('[OK] %s\n', map_base);
    if has_lidar_cov
        fprintf('     Sigma_lidar size: %dx%d\n', size(Sigma_lidar, 1), size(Sigma_lidar, 2));
        fprintf('     Kontrola diagonalni variance\n');
        fprintf('     max|diag(Sigma_lidar)-sigma^2| = %.3e\n', lidar_diag_err);
        fprintf('     Sigma_lidar =\n');
        disp(Sigma_lidar);
    else
        fprintf('     Sigma_lidar: insufficient valid samples\n');
    end
    if has_gnss_cov
        fprintf('     Sigma_gnss  size: %dx%d\n', size(Sigma_gnss, 1), size(Sigma_gnss, 2));
        fprintf('     Kontrola diagonalni variance\n');
        fprintf('     max|diag(Sigma_gnss)-sigma^2| = %.3e\n', gnss_diag_err);
        fprintf('     Sigma_gnss =\n');
        disp(Sigma_gnss);
    else
        fprintf('     Sigma_gnss: insufficient valid samples\n');
    end

    if isfield(s, 'lidar_std') && ~isempty(s.lidar_std)
        lidar_std_old = s.lidar_std;
        fprintf('     mean(std Task2) vs mean(std filtered): %.4g vs %.4g\n', ...
            mean(lidar_std_old), mean(lidar_std_filtered));
    end
    if isfield(s, 'gnss_std') && ~isempty(s.gnss_std)
        gnss_std_old = s.gnss_std;
        fprintf('     mean(std Task2) vs mean(std filtered): %.4g vs %.4g\n', ...
            mean(gnss_std_old, 'omitnan'), mean(gnss_std_filtered, 'omitnan'));
    end
    fprintf('\n');
end

fprintf('Done.\n');
