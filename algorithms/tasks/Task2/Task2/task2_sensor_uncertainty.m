%% Task 2: Sensor Uncertainty Analysis
% This script processes LiDAR and GNSS measurements collected during static
% robot operation and computes sensor noise statistics.

close all; clc;

% Note: Before running this script, execute main.m and let it run for at 
% least 150 iterations while the robot is stationary. Then stop it (Ctrl+C 
% or Stop button) and run this script.

% Check if data is available in workspace
if ~exist('public_vars', 'var') || ~exist('read_only_vars', 'var')
    error('Data not found! Please run main.m first and let it collect data.');
end

% Extract collected data
lidar_data_raw = public_vars.lidar_history;
gnss_data = public_vars.gnss_history;

% Filter out invalid LiDAR data (Inf and NaN)
valid_rows = ~any(isinf(lidar_data_raw) | isnan(lidar_data_raw), 2);
lidar_data = lidar_data_raw(valid_rows, :);
if isempty(lidar_data)
    warning('No valid LiDAR data! Check robot position - may be outside map.');
end

fprintf('\n========== SENSOR UNCERTAINTY ANALYSIS ==========\n');
fprintf('Number of LiDAR samples (valid): %d / %d\n', size(lidar_data, 1), size(lidar_data_raw, 1));
fprintf('Number of GNSS samples: %d\n', size(gnss_data, 1));

%% LiDAR Analysis
fprintf('\n--- LiDAR Measurements (8 Channels) ---\n');
lidar_channels = 8;
lidar_std = std(lidar_data);
lidar_mean = mean(lidar_data);

fprintf('Channel\tMean\t\tStd Dev\n');
fprintf('-------\t-------\t\t-------\n');
for ch = 1:lidar_channels
    fprintf('  %d\t%.2f\t\t%.3g\n', ch, lidar_mean(ch), lidar_std(ch));
end

fprintf('\nLiDAR Statistics:\n');
fprintf('Average Std: %.3g\n', mean(lidar_std));
fprintf('Min Std: %.3g (Channel %d)\n', min(lidar_std), find(lidar_std == min(lidar_std)));
fprintf('Max Std: %.3g (Channel %d)\n', max(lidar_std), find(lidar_std == max(lidar_std)));
fprintf('Std of Stds: %.3g\n', std(lidar_std));

%% GNSS Analysis
fprintf('\n--- GNSS Measurements (X, Y axes) ---\n');
gnss_std = std(gnss_data);
gnss_mean = mean(gnss_data);

fprintf('Axis\tMean\t\tStd Dev\n');
fprintf('----\t-------\t\t-------\n');
fprintf('X\t%.2f\t\t%.3g\n', gnss_mean(1), gnss_std(1));
fprintf('Y\t%.2f\t\t%.3g\n', gnss_mean(2), gnss_std(2));

fprintf('\nGNSS Statistics:\n');
gnss_std_valid = gnss_std(~isnan(gnss_std));
if ~isempty(gnss_std_valid)
    fprintf('Average Std: %.3g\n', mean(gnss_std_valid));
end

%% Visualization: Histograms for LiDAR
if ~isempty(lidar_data)
    figure('Name', 'LiDAR Sensor Noise', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 600]);
    for ch = 1:lidar_channels
        subplot(2, 4, ch);
        
        % Get valid data for this channel
        ch_data = lidar_data(:, ch);
        ch_data_valid = ch_data(~isnan(ch_data) & ~isinf(ch_data));
        
        if ~isempty(ch_data_valid)
            % Histogram with normalization
            h = histogram(ch_data_valid, 25, 'Normalization', 'pdf', 'FaceColor', 'blue', 'FaceAlpha', 0.6);
            hold on;
            
            % Overlay normal distribution (Gaussian PDF)
            x_min = min(ch_data_valid);
            x_max = max(ch_data_valid);
            x_range = linspace(x_min, x_max, 100);
            y_normal = (1 / (lidar_std(ch) * sqrt(2*pi))) * exp(-((x_range - lidar_mean(ch)).^2) / (2*lidar_std(ch)^2));
            plot(x_range, y_normal, 'r-', 'LineWidth', 2, 'DisplayName', 'Normal fit');
            
            xlabel('Distance (m)', 'FontSize', 9);
            ylabel('Density', 'FontSize', 9);
            title(sprintf('Channel %d   |   μ=%.3g, σ=%.3g', ch, lidar_mean(ch), lidar_std(ch)), 'FontSize', 10);
            grid on;
            legend('hide');
            hold off;
        else
            text(0.5, 0.5, sprintf('Channel %d: No valid data', ch), 'HorizontalAlignment', 'center');
        end
    end
    sgtitle('LiDAR Measurements - 8 Channels (with Normal Distribution Overlay)', 'FontSize', 12, 'FontWeight', 'bold');
else
    fprintf('Warning: No valid LiDAR data to plot!\n');
end

%% Visualization: Histograms for GNSS
figure('Name', 'GNSS Sensor Noise', 'NumberTitle', 'off', 'Position', [100, 700, 900, 400]);

subplot(1, 2, 1);
h1 = histogram(gnss_data(:, 1), 25, 'Normalization', 'pdf', 'FaceColor', 'green', 'FaceAlpha', 0.6);
hold on;
x_range_x = linspace(min(gnss_data(:, 1)), max(gnss_data(:, 1)), 100);
y_normal_x = (1 / (gnss_std(1) * sqrt(2*pi))) * exp(-((x_range_x - gnss_mean(1)).^2) / (2*gnss_std(1)^2));
plot(x_range_x, y_normal_x, 'r-', 'LineWidth', 2);
xlabel('Position X (m)', 'FontSize', 10);
ylabel('Density', 'FontSize', 10);
title(sprintf('GNSS X-axis\nμ=%.3g, σ=%.3g', gnss_mean(1), gnss_std(1)), 'FontSize', 11);
grid on;
hold off;

subplot(1, 2, 2);
h2 = histogram(gnss_data(:, 2), 25, 'Normalization', 'pdf', 'FaceColor', 'green', 'FaceAlpha', 0.6);
hold on;
x_range_y = linspace(min(gnss_data(:, 2)), max(gnss_data(:, 2)), 100);
y_normal_y = (1 / (gnss_std(2) * sqrt(2*pi))) * exp(-((x_range_y - gnss_mean(2)).^2) / (2*gnss_std(2)^2));
plot(x_range_y, y_normal_y, 'r-', 'LineWidth', 2);
xlabel('Position Y (m)', 'FontSize', 10);
ylabel('Density', 'FontSize', 10);
title(sprintf('GNSS Y-axis\nμ=%.3g, σ=%.3g', gnss_mean(2), gnss_std(2)), 'FontSize', 11);
grid on;
hold off;

sgtitle('GNSS Measurements - Position Axes (with Normal Distribution Overlay)', 'FontSize', 12, 'FontWeight', 'bold');

%% Create comparison figure
figure('Name', 'Sensor Comparison', 'NumberTitle', 'off');

% LiDAR std deviation comparison
subplot(1, 2, 1);
bar(1:lidar_channels, lidar_std, 'FaceColor', 'blue', 'EdgeColor', 'black');
hold on;
plot([0.5, lidar_channels+0.5], [mean(lidar_std), mean(lidar_std)], 'r--', 'LineWidth', 2);
xlabel('Channel');
ylabel('Standard Deviation (m)');
title('LiDAR Channel Noise Comparison');
grid on;
legend('Channel Std', sprintf('Mean = %.3g', mean(lidar_std)));

% GNSS std deviation comparison
subplot(1, 2, 2);
bar([1, 2], gnss_std, 'FaceColor', 'green', 'EdgeColor', 'black');
set(gca, 'XTickLabel', {'X-axis', 'Y-axis'});
ylabel('Standard Deviation (m)');
title('GNSS Axis Noise Comparison');
grid on;
% Handle NaN values in ylim
valid_gnss = gnss_std(~isnan(gnss_std));
if ~isempty(valid_gnss)
    ylim([0, max(valid_gnss)*1.2]);
end

%% Discussion
fprintf('\n========== ANALYSIS DISCUSSION ==========\n');
fprintf('1. LiDAR Noise Consistency:\n');
std_range = max(lidar_std) - min(lidar_std);
fprintf('   - Standard deviation range: %.3g m\n', std_range);
fprintf('   - Coefficient of variation: %.3g%%\n', (std(lidar_std)/mean(lidar_std))*100);
if std(lidar_std) < 0.1*mean(lidar_std)
    fprintf('   - Conclusion: CONSISTENT across channels\n');
else
    fprintf('   - Conclusion: INCONSISTENT across channels - some channels noisier\n');
end

fprintf('\n2. GNSS Noise Consistency:\n');
gnss_range = max(gnss_std) - min(gnss_std);
fprintf('   - Standard deviation range: %.3g m\n', gnss_range);
if gnss_range < 0.1
    fprintf('   - Conclusion: CONSISTENT between X and Y axes\n');
else
    fprintf('   - Conclusion: INCONSISTENT between X and Y axes\n');
end

fprintf('\n========== END OF ANALYSIS ==========\n');

% Save data for future use
% Find project root (go up until we find main.m)
current_path = fileparts(mfilename('fullpath'));
while ~exist(fullfile(current_path, 'main.m'), 'file')
    parent = fileparts(current_path);
    if strcmp(parent, current_path)  % reached filesystem root
        error('Could not find main.m - cannot locate project root');
    end
    current_path = parent;
end

% Now current_path is project root, setup.m is in algorithms/
setup_file = fullfile(current_path, 'algorithms', 'setup.m');
run(setup_file);  % This defines map_name in workspace

% Save directly in Task2/Task2 folder (where this script is located)
task2_parent = fileparts(mfilename('fullpath')); % .../algorithms/tasks/Task2/Task2

% Extract map base name (without path and extension)
[~, map_base, ~] = fileparts(map_name);

% Create filename with map name
mat_filename = fullfile(task2_parent, sprintf('sensor_uncertainty_data_%s.mat', map_base));
save(mat_filename, 'lidar_data', 'gnss_data', 'lidar_std', 'gnss_std');
fprintf('\nData saved to %s\n', mat_filename);
