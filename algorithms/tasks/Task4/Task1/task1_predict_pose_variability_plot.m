%% Task4/Task1 - predict_pose: variability vs speed/turning
% Runs Monte Carlo sampling and plots how output variance changes
% with forward speed and turning intensity.

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

ro.sampling_period = 0.1;
ro.agent_drive = struct('type', 2, 'interwheel_dist', 0.2, 'max_vel', 1);
old_pose = [2, 8.5, pi/2];
N = 3000;

% Experiment A: forward speed sweep (vR=vL)
v_vals = 0:0.1:0.9;
var_x_fwd = zeros(size(v_vals));
var_y_fwd = zeros(size(v_vals));
var_t_fwd = zeros(size(v_vals));

for i = 1:numel(v_vals)
    mv = [v_vals(i), v_vals(i)];
    samples = zeros(N, 3);
    for k = 1:N
        samples(k, :) = predict_pose(old_pose, mv, ro);
    end
    vxyz = var(samples, 0, 1);
    var_x_fwd(i) = vxyz(1);
    var_y_fwd(i) = vxyz(2);
    var_t_fwd(i) = vxyz(3);
end

% Experiment B: turning sweep (vR=-vL)
u_vals = 0:0.1:0.9;
turn_rate = 2 * u_vals / ro.agent_drive.interwheel_dist;
var_x_turn = zeros(size(u_vals));
var_y_turn = zeros(size(u_vals));
var_t_turn = zeros(size(u_vals));

for i = 1:numel(u_vals)
    mv = [u_vals(i), -u_vals(i)];
    samples = zeros(N, 3);
    for k = 1:N
        samples(k, :) = predict_pose(old_pose, mv, ro);
    end
    vxyz = var(samples, 0, 1);
    var_x_turn(i) = vxyz(1);
    var_y_turn(i) = vxyz(2);
    var_t_turn(i) = vxyz(3);
end

fig = figure('Name', 'predict\_pose variability', 'NumberTitle', 'off', ...
    'Position', [120 100 1400 650], 'Color', 'w');
t = tiledlayout(fig, 1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(v_vals, var_x_fwd, 'r-o', 'LineWidth', 1.8, 'DisplayName', 'Var(x)'); hold on;
plot(v_vals, var_y_fwd, 'b-o', 'LineWidth', 1.8, 'DisplayName', 'Var(y)');
plot(v_vals, var_t_fwd, 'k-o', 'LineWidth', 1.8, 'DisplayName', 'Var(theta)');
grid on;
xlabel('Forward wheel speed v_R=v_L [m/s]');
ylabel('Variance after one step');
title('Variability vs forward speed');
legend('Location', 'northwest', 'Box', 'on');

nexttile;
plot(turn_rate, var_x_turn, 'r-o', 'LineWidth', 1.8, 'DisplayName', 'Var(x)'); hold on;
plot(turn_rate, var_y_turn, 'b-o', 'LineWidth', 1.8, 'DisplayName', 'Var(y)');
plot(turn_rate, var_t_turn, 'k-o', 'LineWidth', 1.8, 'DisplayName', 'Var(theta)');
grid on;
xlabel('Turning rate \omega [rad/s] (v_R=-v_L)');
ylabel('Variance after one step');
title('Variability vs turning rate');
legend('Location', 'northwest', 'Box', 'on');

nexttile;
total_fwd = var_x_fwd + var_y_fwd + var_t_fwd;
total_turn = var_x_turn + var_y_turn + var_t_turn;
plot(v_vals, total_fwd, 'm-o', 'LineWidth', 2.0, 'DisplayName', 'Total variance (forward)'); hold on;
plot(u_vals, total_turn, 'c-o', 'LineWidth', 2.0, 'DisplayName', 'Total variance (turning)');
grid on;
xlabel('Control magnitude [m/s]');
ylabel('Var(x)+Var(y)+Var(theta)');
title('Summary plot');
legend('Location', 'northwest', 'Box', 'on');

title(t, 'Task4/Task1: Monte Carlo variability of predict\_pose');

ax = findall(fig, 'Type', 'axes');
set(ax, 'Color', 'w');

fig_path = fullfile(out_dir, 'task1_predict_pose_variability.fig');
png_path = fullfile(out_dir, 'task1_predict_pose_variability.png');
savefig(fig, fig_path);
exportgraphics(fig, png_path, 'BackgroundColor', 'white', 'Resolution', 150);

tbl_fwd = table(v_vals(:), var_x_fwd(:), var_y_fwd(:), var_t_fwd(:), ...
    'VariableNames', {'v_forward', 'var_x', 'var_y', 'var_theta'});
tbl_turn = table(turn_rate(:), var_x_turn(:), var_y_turn(:), var_t_turn(:), ...
    'VariableNames', {'omega_turn', 'var_x', 'var_y', 'var_theta'});
writetable(tbl_fwd, fullfile(out_dir, 'task1_variability_forward.csv'));
writetable(tbl_turn, fullfile(out_dir, 'task1_variability_turn.csv'));

fprintf('Saved: %s\n', rel_path(fig_path, project_root));
fprintf('Saved: %s\n', rel_path(png_path, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task1_variability_forward.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(out_dir, 'task1_variability_turn.csv'), project_root));

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end
