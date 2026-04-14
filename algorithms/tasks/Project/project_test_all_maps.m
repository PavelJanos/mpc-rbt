%% Project benchmark - all maps
warm_mode_env = getenv('PROJECT_BENCHMARK_WARM');
warm_mode = any(strcmpi(strtrim(warm_mode_env), {'1', 'true', 'on', 'yes'}));
if ~warm_mode
    clearvars -except warm_mode_env warm_mode;
    close all;
end
clc;

project_root = fileparts(mfilename('fullpath'));
while ~exist(fullfile(project_root, 'main.m'), 'file')
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        error('Could not locate project root (main.m).');
    end
    project_root = parent;
end

addpath(genpath(fullfile(project_root, 'algorithms')));
addpath(fullfile(project_root, 'utils'));

out_dir = fileparts(mfilename('fullpath'));
maps_dir = fullfile(project_root, 'maps');
map_files = dir(fullfile(maps_dir, '*.txt'));
all_map_names = sort({map_files.name});

% Benchmark modes:
%   'test' -> run one map only (chosen pseudo-randomly but repeatably)
%   'full' -> run all maps
benchmark_mode = 'test';
benchmark_mode_env = getenv('PROJECT_BENCHMARK_MODE');
if any(strcmpi(benchmark_mode_env, {'test', 'full'}))
    benchmark_mode = lower(benchmark_mode_env);
end

% View mode:
%   true  -> open a live figure during each simulated run
%   false -> run headless and only save post-run outputs
view_mode = true;
view_mode_env = getenv('PROJECT_VIEW_MODE');
if ~isempty(view_mode_env)
    view_mode = any(strcmpi(strtrim(view_mode_env), {'1', 'true', 'on', 'yes'}));
end
shared_live_view_env = getenv('PROJECT_SHARED_LIVE_VIEW');
shared_view_mode = parse_shared_view_mode(getenv('PROJECT_SHARED_VIEW_MODE'));

% Case modes:
%   'regenerate' -> always create a new set of start/goal/theta cases
%   'reuse'      -> load existing cases from CSV; if missing, create them once
case_mode = 'reuse';
case_mode_env = getenv('PROJECT_CASE_MODE');
if any(strcmpi(case_mode_env, {'reuse', 'regenerate'}))
    case_mode = lower(case_mode_env);
end

% Stop modes:
%   'all'          -> process all selected map/repeat cases
%   'first_failure'-> stop after first non-goal result, save partial outputs, and exit
stop_mode = 'all';
stop_mode_env = getenv('PROJECT_STOP_MODE');
if any(strcmpi(stop_mode_env, {'all', 'first_failure'}))
    stop_mode = lower(stop_mode_env);
end
test_map_seed = 2026;

if strcmpi(benchmark_mode, 'test')
    rng(test_map_seed, 'twister');
    selected_map_label = getenv('PROJECT_TEST_MAP');
    if ~isempty(selected_map_label) && any(strcmp(all_map_names, [selected_map_label, '.txt']))
        map_names = {[selected_map_label, '.txt']};
    else
        selected_idx = randi(numel(all_map_names));
        map_names = all_map_names(selected_idx);
        selected_map_label = erase(map_names{1}, '.txt');
    end
    selected_map_label = erase(map_names{1}, '.txt');
    cases_file = fullfile(out_dir, sprintf('project_test_all_maps_cases_test_%s.csv', selected_map_label));
else
    map_names = all_map_names;
    selected_map_label = 'all_maps';
    cases_file = fullfile(out_dir, 'project_test_all_maps_cases_full.csv');
end
cases_file_env = getenv('PROJECT_CASES_FILE');
if ~isempty(cases_file_env)
    cases_file = cases_file_env;
end

num_repeats = 10;
num_repeats_env = getenv('PROJECT_NUM_REPEATS');
if ~isempty(num_repeats_env)
    num_repeats_val = str2double(num_repeats_env);
    if isfinite(num_repeats_val) && num_repeats_val >= 1
        num_repeats = max(1, round(num_repeats_val));
    end
end
timeout_steps = 2050;
timeout_steps_env = getenv('PROJECT_TIMEOUT_STEPS');
if ~isempty(timeout_steps_env)
    timeout_steps_val = str2double(timeout_steps_env);
    if isfinite(timeout_steps_val) && timeout_steps_val >= 1
        timeout_steps = max(1, round(timeout_steps_val));
    end
end
concurrent_runs = 1;
concurrent_runs_env = getenv('PROJECT_CONCURRENT_RUNS');
if ~isempty(concurrent_runs_env)
    concurrent_runs_val = str2double(concurrent_runs_env);
    if isfinite(concurrent_runs_val) && concurrent_runs_val >= 1
        concurrent_runs = max(1, round(concurrent_runs_val));
    end
end
use_process_parallel = false;
if concurrent_runs > 1 && ~supports_parallel_runs()
    if supports_process_parallel_runs()
        warning('Parallel Computing Toolbox not available. Using process-based benchmark parallelism via MATLAB R2023b workers.');
        use_process_parallel = true;
    else
        warning('Parallel Computing Toolbox not available and no MATLAB R2023b worker executable found. Falling back to serial benchmark execution.');
        concurrent_runs = 1;
    end
end
session_label = getenv('PROJECT_SESSION_LABEL');
sessions_root = fullfile(out_dir, 'RunSessions');
session_dir = create_session_dir(sessions_root, 'benchmark', session_label);
history_dir = fullfile(session_dir, 'HistoryMapsReview');
if ~exist(history_dir, 'dir')
    mkdir(history_dir);
end
rows = cell(numel(map_names) * num_repeats, 18);
row_idx = 1;
stop_requested = false;
stop_reason = "";
stop_map_name = "";
stop_repeat_id = nan;
cases_tbl = load_or_create_cases(map_names, maps_dir, num_repeats, case_mode, cases_file);
tasks = build_benchmark_tasks(cases_tbl);
shared_live_view = view_mode && numel(tasks) > 1 && numel(unique(string(cases_tbl.map_name), 'stable')) == 1;
if ~isempty(shared_live_view_env)
    shared_live_view = shared_live_view && any(strcmpi(strtrim(shared_live_view_env), {'1', 'true', 'on', 'yes'}));
end
dashboard = [];
multi_run_view = view_mode && concurrent_runs > 1 && ~shared_live_view;
if multi_run_view
    dashboard = init_benchmark_dashboard(tasks, timeout_steps, concurrent_runs);
end
task_results = cell(numel(tasks), 1);

if concurrent_runs <= 1
    for t = 1:numel(tasks)
        task_result = execute_benchmark_case(tasks(t), maps_dir, history_dir, timeout_steps, view_mode && ~shared_live_view);
        rows(row_idx, :) = benchmark_row_from_result(task_result); %#ok<AGROW>
        task_results{t} = task_result;
        if multi_run_view
            dashboard = update_benchmark_dashboard(dashboard, t, task_result);
        end
        row_idx = row_idx + 1;
        if strcmp(stop_mode, 'first_failure') && ~strcmp(string(task_result.status), "goal")
            stop_requested = true;
            stop_reason = string(task_result.status);
            stop_map_name = string(task_result.map_name);
            stop_repeat_id = task_result.repeat_id;
            break;
        end
    end
elseif use_process_parallel
    [task_results, rows, row_idx, stop_requested, stop_reason, stop_map_name, stop_repeat_id, dashboard] = execute_benchmark_tasks_process_parallel( ...
        tasks, maps_dir, history_dir, timeout_steps, stop_mode, session_dir, concurrent_runs, dashboard, multi_run_view, project_root);
else
    pool = gcp('nocreate');
    target_workers = min(concurrent_runs, numel(tasks));
    if isempty(pool) || pool.NumWorkers ~= target_workers
        if ~isempty(pool)
            delete(pool);
        end
        pool = parpool('local', target_workers);
    end
    future_struct = struct('future', {}, 'task_idx', {});
    next_task_idx = 1;
    while next_task_idx <= numel(tasks) || ~isempty(future_struct)
        while next_task_idx <= numel(tasks) && numel(future_struct) < target_workers && ~stop_requested
            fut = parfeval(pool, @execute_benchmark_case, 1, tasks(next_task_idx), maps_dir, history_dir, timeout_steps, false);
            future_struct(end + 1).future = fut; %#ok<AGROW>
            future_struct(end).task_idx = next_task_idx;
            if multi_run_view
                dashboard = update_benchmark_dashboard_status(dashboard, next_task_idx, "running", nan, nan, "");
            end
            next_task_idx = next_task_idx + 1;
        end
        if isempty(future_struct)
            break;
        end
        future_list = [future_struct.future];
        [completed_idx, task_result] = fetchNext(future_list);
        task_idx = future_struct(completed_idx).task_idx;
        future_struct(completed_idx) = [];
        rows(row_idx, :) = benchmark_row_from_result(task_result); %#ok<AGROW>
        task_results{task_idx} = task_result;
        if multi_run_view
            dashboard = update_benchmark_dashboard(dashboard, task_idx, task_result);
        end
        row_idx = row_idx + 1;
        if strcmp(stop_mode, 'first_failure') && ~strcmp(string(task_result.status), "goal")
            stop_requested = true;
            stop_reason = string(task_result.status);
            stop_map_name = string(task_result.map_name);
            stop_repeat_id = task_result.repeat_id;
            for k = 1:numel(future_struct)
                cancel(future_struct(k).future);
            end
            future_struct = struct('future', {}, 'task_idx', {});
        end
    end
end

if shared_live_view
    play_shared_benchmark_live_view(tasks, task_results, maps_dir, timeout_steps, shared_view_mode);
end

detail_tbl = cell2table(rows(1:max(row_idx - 1, 0), :), 'VariableNames', ...
    {'map_name', 'repeat_id', 'start_x', 'start_y', 'start_theta', 'goal_x', 'goal_y', 'max_vel', 'cases_mode', ...
     'status', 'success', 'steps', 'goal_error_m', 'travelled_m', 'runtime_s', ...
     'final_x', 'final_y', 'final_theta'});

summary_map_names = map_names;
if ~isempty(detail_tbl) && height(detail_tbl) > 0
    available_names = unique(string(detail_tbl.map_name), 'stable');
    summary_map_names = map_names(ismember(erase(map_names, '.txt'), cellstr(available_names)));
end
summary_tbl = summarize_results(detail_tbl, summary_map_names);

writetable(detail_tbl, fullfile(session_dir, 'project_test_all_maps_detail.csv'));
writetable(summary_tbl, fullfile(session_dir, 'project_test_all_maps_summary.csv'));

fig = figure('Name', 'Project benchmark - all maps', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [100 100 1300 700]);
fig.ToolBar = 'none';
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
bar(categorical(summary_tbl.map_name), summary_tbl.success_rate, 0.55, ...
    'FaceColor', [0.15 0.55 0.90], 'EdgeColor', [0.10 0.25 0.45]);
ylim([0 1]);
ylabel('Success rate [-]');
title('Project benchmark: success rate per map', 'FontWeight', 'bold');
grid on;

nexttile;
yyaxis left;
bar(categorical(summary_tbl.map_name), summary_tbl.mean_steps_success, 0.45, ...
    'FaceColor', [0.20 0.70 0.30], 'EdgeColor', [0.10 0.40 0.15]);
ylabel('Mean steps (success only) [-]');
yyaxis right;
plot(categorical(summary_tbl.map_name), summary_tbl.mean_goal_error_m, 'x-', ...
    'Color', [0.85 0.33 0.10], 'LineWidth', 2, 'MarkerSize', 10);
ylabel('Mean goal error [m]');
title('Project benchmark: efficiency and final error', 'FontWeight', 'bold');
grid on;

exportgraphics(fig, fullfile(session_dir, 'project_test_all_maps.png'), ...
    'BackgroundColor', 'white', 'Resolution', 170);
savefig(fig, fullfile(session_dir, 'project_test_all_maps.fig'));

write_report(fullfile(session_dir, 'project_test_all_maps_report.md'), detail_tbl, summary_tbl, benchmark_mode, selected_map_label, view_mode, timeout_steps, stop_mode, stop_requested, stop_reason, stop_map_name, stop_repeat_id);

fprintf('Session: %s\n', rel_path(session_dir, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(session_dir, 'project_test_all_maps_detail.csv'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(session_dir, 'project_test_all_maps_summary.csv'), project_root));
fprintf('Saved: %s\n', rel_path(cases_file, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(session_dir, 'project_test_all_maps.png'), project_root));
fprintf('Saved: %s\n', rel_path(fullfile(session_dir, 'project_test_all_maps_report.md'), project_root));

function result = simulate_single_run(map, discrete_map, start_pose, max_steps, view_mode)
tic;

% Ensure controller persistent state cannot leak between benchmark runs.
clear plan_motion

public_vars = struct();
public_vars.motion_vector = [0, 0];
public_vars.init_iterations = 1;
public_vars.pf_enabled = 0;
public_vars.kf_enabled = 0;
public_vars.estimated_pose = [];
public_vars.path = [];
public_vars.particles = [];
public_vars.force_grid_planner = true;
public_vars.path_planner_mode = 'astar';
public_vars.path_smoothing_mode = 'chaikin';
public_vars.path_clearance_m = 0.25;
public_vars.path_resample_ds_m = 0.10;
public_vars.path_refine_iters = 2;
public_vars.smooth_chaikin_iters = 4;
public_vars.controller_mode = 'pure_pursuit';

private_vars = struct();
private_vars.agent_pose = start_pose;
private_vars.raycasts = [];
private_vars.agent_position_history = [];

read_only_vars = struct();
read_only_vars.map = map;
read_only_vars.discrete_map = discrete_map;
read_only_vars.agent_drive.type = 2;
read_only_vars.agent_drive.interwheel_dist = 0.2;
read_only_vars.agent_drive.max_vel = 1;
read_only_vars.measurement_distances = [];
read_only_vars.gnss_position = [];
read_only_vars.lidar_config = [0, 45, 90, 135, 180, 225, 270, 315] / 180 * pi;
read_only_vars.sampling_period = 0.1;
read_only_vars.max_particles = 1000;
read_only_vars.counter = 1;
read_only_vars.est_position_history = nan(1, 3);
read_only_vars.gnss_history = [];

live_view = [];
if view_mode
    live_view = init_live_view(map, private_vars.agent_pose);
end

result = struct();
result.status = "timeout";
result.success = false;
result.steps = 0;
result.goal_error_m = norm(start_pose(1:2) - map.goal(1:2));
result.travelled_m = 0;
result.final_pose = start_pose;

while true
    if is_in_goal(private_vars, read_only_vars)
        result.status = "goal";
        result.success = true;
        break;
    end

    if is_in_wall(private_vars, read_only_vars)
        result.status = "wall";
        break;
    end

    if is_out(private_vars, read_only_vars)
        result.status = "out";
        break;
    end

    if size(public_vars.particles, 1) > read_only_vars.max_particles
        result.status = "too_many_particles";
        break;
    end

    if read_only_vars.counter > max_steps
        result.status = "timeout";
        break;
    end

    [read_only_vars.lidar_distances, private_vars.raycasts] = ...
        lidar_measure(read_only_vars.map, private_vars.agent_pose, read_only_vars.lidar_config);
    read_only_vars.gnss_position = gnss_measure(private_vars.agent_pose, read_only_vars.map.gnss_denied);
    read_only_vars.gnss_history = [read_only_vars.gnss_history; read_only_vars.gnss_position];
    read_only_vars.mocap_pose = mocap_measure(private_vars.agent_pose, read_only_vars.map.gnss_denied);

    public_vars = student_workspace(read_only_vars, public_vars);
    read_only_vars.est_position_history = [read_only_vars.est_position_history; public_vars.estimated_pose];

    if ~isfield(public_vars, 'motion_vector') || numel(public_vars.motion_vector) ~= 2 ...
            || ~all(isfinite(public_vars.motion_vector))
        result.status = "invalid_motion";
        break;
    end

    private_vars.agent_pose = move_agent(private_vars.agent_pose, public_vars.motion_vector, ...
        read_only_vars.agent_drive, read_only_vars.sampling_period);
    private_vars.agent_position_history = [private_vars.agent_position_history; private_vars.agent_pose];
    if view_mode && ~isempty(live_view) && isgraphics(live_view.figure)
        live_view = update_live_view(live_view, private_vars, read_only_vars, public_vars);
    end
    read_only_vars.counter = read_only_vars.counter + 1;
end

result.runtime_s = toc;
result.steps = max(read_only_vars.counter - 1, 0);
result.final_pose = private_vars.agent_pose;
result.goal_error_m = norm(private_vars.agent_pose(1:2) - map.goal(1:2));
result.path = [];
if isfield(public_vars, 'path') && ~isempty(public_vars.path)
    result.path = public_vars.path;
end
result.pose_history = private_vars.agent_position_history;
result.est_history = read_only_vars.est_position_history;
if size(private_vars.agent_position_history, 1) >= 2
    result.travelled_m = sum(vecnorm(diff(private_vars.agent_position_history(:, 1:2), 1, 1), 2, 2));
else
    result.travelled_m = 0;
end

if view_mode && ~isempty(live_view) && isgraphics(live_view.figure)
    title(live_view.axes, sprintf('%s | steps=%d | goal err=%.3f m', ...
        result.status, result.steps, result.goal_error_m), 'Interpreter', 'none', 'FontWeight', 'bold');
    drawnow;
end
end

function tasks = build_benchmark_tasks(cases_tbl)
tasks = repmat(struct( ...
    'map_name', "", ...
    'repeat_id', 0, ...
    'start_pose', [nan, nan, nan], ...
    'goal_xy', [nan, nan], ...
    'cases_mode', "", ...
    'max_vel', 1.0), height(cases_tbl), 1);
for i = 1:height(cases_tbl)
    tasks(i).map_name = string(cases_tbl.map_name(i));
    tasks(i).repeat_id = cases_tbl.repeat_id(i);
    tasks(i).start_pose = [cases_tbl.start_x(i), cases_tbl.start_y(i), cases_tbl.start_theta(i)];
    tasks(i).goal_xy = [cases_tbl.goal_x(i), cases_tbl.goal_y(i)];
    if ismember('cases_mode', cases_tbl.Properties.VariableNames)
        tasks(i).cases_mode = string(cases_tbl.cases_mode(i));
    else
        tasks(i).cases_mode = "reuse";
    end
    tasks(i).max_vel = 1.0;
end
end

function task_result = execute_benchmark_case(task, maps_dir, history_dir, timeout_steps, view_mode)
map_path = fullfile(maps_dir, [char(task.map_name), '.txt']);
map = load_map(map_path);
map.discretization_step = 0.2;
map.goal_tolerance = 0.5;
discrete_map = generate_discrete_map(map);
map.goal = task.goal_xy;
start_pose = task.start_pose;
result = simulate_single_run(map, discrete_map, start_pose, timeout_steps, view_mode);
save_run_review_figure(history_dir, char(task.map_name), task.repeat_id, map, start_pose, result);
task_result = struct( ...
    'map_name', string(task.map_name), ...
    'repeat_id', task.repeat_id, ...
    'start_pose', start_pose, ...
    'goal_xy', task.goal_xy, ...
    'max_vel', task.max_vel, ...
    'cases_mode', string(task.cases_mode), ...
    'status', string(result.status), ...
    'success', logical(result.success), ...
    'steps', result.steps, ...
    'goal_error_m', result.goal_error_m, ...
    'travelled_m', result.travelled_m, ...
    'runtime_s', result.runtime_s, ...
    'final_pose', result.final_pose, ...
    'path', result.path, ...
    'pose_history', result.pose_history, ...
    'est_history', result.est_history);
artifact_file = benchmark_replay_artifact_file(history_dir, char(task.map_name), task.repeat_id);
task_result_artifact = task_result; %#ok<NASGU>
save(artifact_file, 'task_result_artifact', '-v7');
end

function row = benchmark_row_from_result(task_result)
row = { ...
    char(task_result.map_name), ...
    task_result.repeat_id, ...
    task_result.start_pose(1), ...
    task_result.start_pose(2), ...
    task_result.start_pose(3), ...
    task_result.goal_xy(1), ...
    task_result.goal_xy(2), ...
    task_result.max_vel, ...
    string(task_result.cases_mode), ...
    string(task_result.status), ...
    task_result.success, ...
    task_result.steps, ...
    task_result.goal_error_m, ...
    task_result.travelled_m, ...
    task_result.runtime_s, ...
    task_result.final_pose(1), ...
    task_result.final_pose(2), ...
    task_result.final_pose(3)};
end

function artifact_file = benchmark_replay_artifact_file(history_dir, map_name, repeat_id)
artifact_file = fullfile(history_dir, sprintf('benchmark_%s_run_%02d_replay.mat', map_name, repeat_id));
end

function dashboard = init_benchmark_dashboard(tasks, timeout_steps, concurrent_runs)
fig = figure('Name', 'Project benchmark batch dashboard', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [120 120 900 560]);
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.93 0.94 0.05], ...
    'BackgroundColor', 'w', 'HorizontalAlignment', 'left', ...
    'FontName', 'Consolas', 'FontSize', 12, ...
    'String', sprintf('Concurrent runs: %d | Timeout: %d steps | Tasks: %d', concurrent_runs, timeout_steps, numel(tasks)));
data = cell(numel(tasks), 7);
for i = 1:numel(tasks)
    data{i,1} = char(tasks(i).map_name);
    data{i,2} = tasks(i).repeat_id;
    data{i,3} = 'pending';
    data{i,4} = '';
    data{i,5} = NaN;
    data{i,6} = NaN;
    data{i,7} = '';
end
tbl = uitable(fig, 'Units', 'normalized', 'Position', [0.03 0.05 0.94 0.86], ...
    'Data', data, ...
    'ColumnName', {'Map', 'Repeat', 'State', 'Result', 'Steps', 'GoalErr', 'Note'}, ...
    'ColumnWidth', {120, 60, 80, 90, 80, 90, 280});
dashboard = struct('figure', fig, 'table', tbl, 'data', {data});
drawnow;
end

function dashboard = update_benchmark_dashboard_status(dashboard, task_idx, state_txt, steps, goal_err, note_txt)
if isempty(dashboard) || ~isgraphics(dashboard.figure)
    return;
end
dashboard.data{task_idx, 3} = char(string(state_txt));
dashboard.data{task_idx, 4} = '';
dashboard.data{task_idx, 5} = steps;
dashboard.data{task_idx, 6} = goal_err;
dashboard.data{task_idx, 7} = char(string(note_txt));
dashboard.table.Data = dashboard.data;
drawnow limitrate;
end

function dashboard = update_benchmark_dashboard(dashboard, task_idx, task_result)
if isempty(dashboard) || ~isgraphics(dashboard.figure)
    return;
end
dashboard.data{task_idx, 3} = 'done';
dashboard.data{task_idx, 4} = char(task_result.status);
dashboard.data{task_idx, 5} = task_result.steps;
dashboard.data{task_idx, 6} = task_result.goal_error_m;
dashboard.data{task_idx, 7} = sprintf('travel %.2f m | runtime %.2f s', task_result.travelled_m, task_result.runtime_s);
dashboard.table.Data = dashboard.data;
drawnow limitrate;
end

function session_dir = create_session_dir(sessions_root, prefix, session_label)
if ~exist(sessions_root, 'dir')
    mkdir(sessions_root);
end
if isempty(session_label)
    session_label = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
else
    session_label = regexprep(char(session_label), '[^A-Za-z0-9._-]', '_');
end
session_dir = fullfile(sessions_root, sprintf('%s_%s', prefix, session_label));
suffix = 1;
base_dir = session_dir;
while exist(session_dir, 'dir')
    session_dir = sprintf('%s_%02d', base_dir, suffix);
    suffix = suffix + 1;
end
mkdir(session_dir);
end

function tf = supports_parallel_runs()
tf = exist('parpool', 'file') == 2 && exist('parfeval', 'file') == 2 && license('test', 'Distrib_Computing_Toolbox');
end

function tf = supports_process_parallel_runs()
tf = exist(get_matlab_r2023b_executable(), 'file') == 2;
end

function exe_path = get_matlab_r2023b_executable()
exe_path = 'C:\Program Files\MATLAB\R2023b\bin\matlab.exe';
if exist(exe_path, 'file') ~= 2
    exe_path = fullfile(matlabroot, 'bin', 'matlab.exe');
end
end

function [task_results, rows, row_idx, stop_requested, stop_reason, stop_map_name, stop_repeat_id, dashboard] = execute_benchmark_tasks_process_parallel(tasks, maps_dir, history_dir, timeout_steps, stop_mode, session_dir, concurrent_runs, dashboard, multi_run_view, project_root)
task_results = cell(numel(tasks), 1);
rows = cell(numel(tasks), 18);
row_idx = 1;
stop_requested = false;
stop_reason = "";
stop_map_name = "";
stop_repeat_id = nan;
worker_root = fullfile(session_dir, 'process_workers');
if ~exist(worker_root, 'dir')
    mkdir(worker_root);
end
target_workers = min(concurrent_runs, numel(tasks));
active = struct('task_idx', {}, 'result_file', {}, 'log_file', {});
next_task_idx = 1;
while next_task_idx <= numel(tasks) || ~isempty(active)
    while next_task_idx <= numel(tasks) && numel(active) < target_workers && ~stop_requested
        task_idx = next_task_idx;
        child_label = sprintf('proc_%s_r%02d_%02d_%d', char(tasks(task_idx).map_name), tasks(task_idx).repeat_id, task_idx, randi(1e6));
        child_session_dir = fullfile(fileparts(session_dir), sprintf('benchmark_%s', child_label));
        child_cases_file = fullfile(worker_root, sprintf('benchmark_case_%02d.csv', task_idx));
        task_tbl = table(string(tasks(task_idx).map_name), tasks(task_idx).repeat_id, ...
            tasks(task_idx).start_pose(1), tasks(task_idx).start_pose(2), tasks(task_idx).start_pose(3), ...
            tasks(task_idx).goal_xy(1), tasks(task_idx).goal_xy(2), tasks(task_idx).max_vel, ...
            string(tasks(task_idx).cases_mode), ...
            'VariableNames', {'map_name','repeat_id','start_x','start_y','start_theta','goal_x','goal_y','max_vel','cases_mode'});
        writetable(task_tbl, child_cases_file);
        task_file = fullfile(worker_root, sprintf('benchmark_task_%02d.mat', task_idx));
        result_file = fullfile(worker_root, sprintf('benchmark_result_%02d.mat', task_idx));
        log_file = fullfile(worker_root, sprintf('benchmark_worker_%02d.log', task_idx));
        spec = struct( ...
            'project_dir', fileparts(mfilename('fullpath')), ...
            'session_label', child_label, ...
            'session_dir', child_session_dir, ...
            'cases_file', child_cases_file, ...
            'map_name', char(tasks(task_idx).map_name), ...
            'repeat_id', tasks(task_idx).repeat_id, ...
            'timeout_steps', timeout_steps, ...
            'project_root', project_root, ...
            'history_dir', history_dir, ...
            'maps_dir', maps_dir);
        save(task_file, 'spec');
        launch_matlab_worker(task_file, result_file, log_file, 'project_benchmark_process_worker');
        active(end + 1) = struct('task_idx', task_idx, 'result_file', result_file, 'log_file', log_file); %#ok<AGROW>
        if multi_run_view
            dashboard = update_benchmark_dashboard_status(dashboard, task_idx, "running", nan, nan, "");
        end
        next_task_idx = next_task_idx + 1;
    end

    pause(0.5);
    keep_mask = true(1, numel(active));
    for i = 1:numel(active)
        if exist(active(i).result_file, 'file') == 2
            loaded = try_load_worker_result(active(i).result_file);
            if ~isfield(loaded, 'worker_result')
                continue;
            end
            worker_result = loaded.worker_result;
            task_idx = active(i).task_idx;
            if worker_result.ok
                task_result = worker_result.task_result;
            else
                task_result = failed_benchmark_task_result(tasks(task_idx), worker_result, active(i).log_file);
            end
            task_results{task_idx} = task_result;
            rows(row_idx, :) = benchmark_row_from_result(task_result); %#ok<AGROW>
            if multi_run_view
                dashboard = update_benchmark_dashboard(dashboard, task_idx, task_result);
            end
            row_idx = row_idx + 1;
            if strcmp(stop_mode, 'first_failure') && ~strcmp(string(task_result.status), "goal")
                stop_requested = true;
                stop_reason = string(task_result.status);
                stop_map_name = string(task_result.map_name);
                stop_repeat_id = task_result.repeat_id;
            end
            keep_mask(i) = false;
        end
    end
    active = active(keep_mask);
end
end

function task_result = failed_benchmark_task_result(task, worker_result, log_file)
status_txt = "worker_error";
if isfield(worker_result, 'status') && strlength(string(worker_result.status)) > 0
    status_txt = string(worker_result.status);
end
task_result = struct( ...
    'map_name', string(task.map_name), ...
    'repeat_id', task.repeat_id, ...
    'start_pose', task.start_pose, ...
    'goal_xy', task.goal_xy, ...
    'max_vel', task.max_vel, ...
    'cases_mode', string(task.cases_mode), ...
    'status', status_txt, ...
    'success', false, ...
    'steps', nan, ...
    'goal_error_m', inf, ...
    'travelled_m', nan, ...
    'runtime_s', nan, ...
    'final_pose', [nan nan nan], ...
    'path', zeros(0,2), ...
    'pose_history', zeros(0,3), ...
    'est_history', zeros(0,3));
if isfield(worker_result, 'message') && ~isempty(worker_result.message)
    warning('Benchmark worker failed for %s run %d: %s. Log: %s', char(task.map_name), task.repeat_id, worker_result.message, log_file);
end
end

function launch_matlab_worker(task_file, result_file, log_file, worker_fn)
matlab_exe = get_matlab_r2023b_executable();
project_dir = fileparts(mfilename('fullpath'));
batch_cmd = sprintf('cd(''%s''); %s(''%s'',''%s'');', ...
    strrep(project_dir, '\', '/'), worker_fn, strrep(task_file, '\', '/'), strrep(result_file, '\', '/'));
sys_cmd = sprintf('cmd /c start "" /B "%s" -batch "%s" > "%s" 2>&1', ...
    matlab_exe, batch_cmd, log_file);
[status, msg] = system(sys_cmd);
if status ~= 0
    error('Failed to launch MATLAB worker: %s', msg);
end
end

function loaded = try_load_worker_result(result_file)
loaded = struct();
for attempt = 1:12
    try
        loaded = load(result_file, 'worker_result');
        return;
    catch
        pause(0.25);
    end
end
end

function play_shared_benchmark_live_view(tasks, task_results, maps_dir, timeout_steps, shared_view_mode)
if isempty(task_results)
    return;
end
valid_mask = cellfun(@(x) isstruct(x) && ~isempty(x), task_results);
task_results = task_results(valid_mask);
if isempty(task_results)
    return;
end
map_path = fullfile(maps_dir, [char(tasks(1).map_name), '.txt']);
map = load_map(map_path);
map.discretization_step = 0.2;
map.goal_tolerance = 0.5;
fig = figure('Name', 'Project benchmark shared live view', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [80 80 1700 920]);
fig.ToolBar = 'figure';
ax = axes(fig, 'Position', [0.05 0.08 0.70 0.86]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
axis(ax, [map.limits(1)-1 map.limits(3)+1 map.limits(2)-1 map.limits(4)+1]);
title(ax, sprintf('Project benchmark shared live view | %s | %d runs', char(tasks(1).map_name), numel(task_results)), 'FontWeight', 'bold');
for i = 1:size(map.walls, 1)
    line(ax, [map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], 'Color', 'black', 'LineWidth', 5);
end
if isfield(map, 'gnss_denied') && ~isempty(map.gnss_denied)
    for i = 1:size(map.gnss_denied, 1)
        pgon = polyshape(map.gnss_denied(i, 1:2:end), map.gnss_denied(i, 2:2:end));
        plot(ax, pgon, 'FaceColor', 'magenta', 'FaceAlpha', 0.08, 'EdgeColor', 'magenta', 'LineStyle', '--');
    end
end
colors = lines(max(numel(task_results), 3));
show_extras = strcmp(shared_view_mode, 'full');
handles = repmat(struct('traj', [], 'est', [], 'path', [], 'agent', [], 'goal', [], 'label', []), numel(task_results), 1);
max_steps_seen = 0;
for i = 1:numel(task_results)
    tr = task_results{i};
    plot(ax, tr.start_pose(1), tr.start_pose(2), 'o', 'Color', colors(i,:), 'MarkerSize', 6, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    [gx, gy] = local_create_circle(tr.goal_xy(1), tr.goal_xy(2), map.goal_tolerance);
    handles(i).goal = plot(ax, gx, gy, '--', 'Color', colors(i,:), 'LineWidth', 1.4, 'HandleVisibility', 'off');
    if show_extras && isfield(tr, 'path') && ~isempty(tr.path)
        handles(i).path = plot(ax, tr.path(:,1), tr.path(:,2), '--', 'Color', colors(i,:) * 0.75, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    else
        handles(i).path = plot(ax, nan, nan, '--', 'Color', colors(i,:) * 0.75, 'LineWidth', 1.2, 'Visible', 'off', 'HandleVisibility', 'off');
    end
    handles(i).traj = plot(ax, nan, nan, '-', 'Color', colors(i,:), 'LineWidth', 2.0, 'DisplayName', sprintf('run %02d true', tr.repeat_id));
    handles(i).est = plot(ax, nan, nan, ':', 'Color', colors(i,:) * 0.65, 'LineWidth', 1.2, ...
        'Visible', onoff(show_extras), 'HandleVisibility', 'off');
    handles(i).agent = plot(ax, nan, nan, '-', 'Color', colors(i,:), 'LineWidth', 2.2, 'HandleVisibility', 'off');
    handles(i).label = text(ax, tr.start_pose(1) + 0.06, tr.start_pose(2) + 0.06, sprintf('R%02d', tr.repeat_id), ...
        'Color', colors(i,:), 'FontSize', 9, 'FontWeight', 'bold', 'Clipping', 'on');
    max_steps_seen = max(max_steps_seen, size(tr.pose_history, 1));
end
legend(ax, 'Location', 'eastoutside');
panel = uipanel(fig, 'Title', 'Shared Run Diagnostics', 'FontWeight', 'bold', ...
    'BackgroundColor', 'white', 'Position', [0.77 0.08 0.22 0.86]);
status_text = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.02 0.92 0.96], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'white', ...
    'FontName', 'Consolas', 'FontSize', 11, 'String', '');
for step_idx = 1:max_steps_seen
    if ~isgraphics(fig)
        return;
    end
    txt = {sprintf('%-12s %5d', 'step:', step_idx), sprintf('%-12s %5d', 'timeout:', timeout_steps), sprintf('%-12s %s', 'mode:', shared_view_mode), ' '};
    for i = 1:numel(task_results)
        tr = task_results{i};
        idx = min(step_idx, size(tr.pose_history, 1));
        poses = tr.pose_history;
        set(handles(i).traj, 'XData', poses(1:idx,1), 'YData', poses(1:idx,2));
        if show_extras
            est = tr.est_history;
            if ~isempty(est)
                est = est(all(isfinite(est(:,1:2)), 2), :);
                if ~isempty(est)
                    est_idx = min(idx, size(est,1));
                    set(handles(i).est, 'XData', est(1:est_idx,1), 'YData', est(1:est_idx,2), 'Visible', 'on');
                end
            end
        end
        [axp, ayp] = local_create_arrow(poses(idx,1:2), poses(idx,3), 0.42);
        set(handles(i).agent, 'XData', axp, 'YData', ayp);
        set(handles(i).label, 'Position', [poses(idx,1) + 0.06, poses(idx,2) + 0.06, 0]);
        state_txt = 'running';
        if idx >= size(tr.pose_history, 1)
            state_txt = char(tr.status);
        end
        goal_err = hypot(poses(idx,1) - tr.goal_xy(1), poses(idx,2) - tr.goal_xy(2));
        txt{end+1} = sprintf('R%02d %-8s steps=%4d', tr.repeat_id, state_txt, min(step_idx, tr.steps)); %#ok<AGROW>
        txt{end+1} = sprintf('  xy=%5.2f,%5.2f ge=%5.2f', poses(idx,1), poses(idx,2), goal_err); %#ok<AGROW>
    end
    status_text.String = sprintf('%s\n', txt{:});
    drawnow limitrate;
end
end

function mode_txt = parse_shared_view_mode(raw_txt)
mode_txt = 'full';
if isempty(raw_txt)
    return;
end
candidate = lower(strtrim(char(raw_txt)));
if any(strcmp(candidate, {'true_only', 'full'}))
    mode_txt = candidate;
end
end

function txt = onoff(tf)
if tf
    txt = 'on';
else
    txt = 'off';
end
end

function start_pose = choose_start_pose(map, discrete_map)
grid = discrete_map.map;
[ny, nx] = size(grid);
limits = discrete_map.limits;

margin = 2;
valid = false(size(grid));
for y = 1 + margin:ny - margin
    for x = 1 + margin:nx - margin
        if grid(y, x) ~= 0
            continue;
        end
        if all(grid(y-margin:y+margin, x-margin:x+margin) == 0, 'all')
            valid(y, x) = true;
        end
    end
end

if ~any(valid, 'all')
    valid = (grid == 0);
end

[ys, xs] = find(valid);
pts = zeros(numel(xs), 2);
for i = 1:numel(xs)
    pts(i, :) = discrete_to_world_local(xs(i), ys(i), limits, [nx ny]);
end

goal = map.goal(1:2);
lower_left = limits(1:2);
score = 1.2 * vecnorm(pts - lower_left, 2, 2) - 0.15 * vecnorm(pts - goal, 2, 2);
score(vecnorm(pts - goal, 2, 2) < 1.5) = inf;

[~, idx] = min(score);
start_xy = pts(idx, :);
start_theta = atan2(goal(2) - start_xy(2), goal(1) - start_xy(1));
start_pose = [start_xy, start_theta];
end

function xy = discrete_to_world_local(ix, iy, limits, dims)
nx = dims(1);
ny = dims(2);

if nx <= 1
    x = limits(1);
else
    x = limits(1) + (ix - 1) * (limits(3) - limits(1)) / (nx - 1);
end

if ny <= 1
    y = limits(2);
else
    y = limits(2) + (iy - 1) * (limits(4) - limits(2)) / (ny - 1);
end

xy = [x, y];
end

function summary_tbl = summarize_results(detail_tbl, map_names)
map_labels = erase(map_names(:), '.txt');
rows = cell(numel(map_labels), 1);
for i = 1:numel(map_labels)
    name = map_labels{i};
    idx = strcmp(detail_tbl.map_name, name);
    subt = detail_tbl(idx, :);
    if isempty(subt) || height(subt) == 0
        rows{i} = [];
        continue;
    end

    success_only = subt.success;
    if any(success_only)
        mean_steps_success = mean(subt.steps(success_only));
    else
        mean_steps_success = nan;
    end

    rows{i} = { ...
        name, ...
        mean(subt.success), ...
        sum(strcmp(subt.status, "goal")), ...
        sum(strcmp(subt.status, "wall")), ...
        sum(strcmp(subt.status, "out")), ...
        sum(strcmp(subt.status, "timeout")), ...
        mean_steps_success, ...
        mean(subt.goal_error_m), ...
        mean(subt.runtime_s), ...
        subt.goal_x(1), ...
        subt.goal_y(1), ...
        subt.max_vel(1)};
end

rows = rows(~cellfun(@isempty, rows));
if isempty(rows)
    summary_tbl = cell2table(cell(0, 12), 'VariableNames', ...
        {'map_name', 'success_rate', 'goal_runs', 'wall_runs', 'out_runs', 'timeout_runs', ...
         'mean_steps_success', 'mean_goal_error_m', 'mean_runtime_s', 'goal_x', 'goal_y', 'max_vel'});
    return;
end

summary_tbl = cell2table(vertcat(rows{:}), 'VariableNames', ...
    {'map_name', 'success_rate', 'goal_runs', 'wall_runs', 'out_runs', 'timeout_runs', ...
     'mean_steps_success', 'mean_goal_error_m', 'mean_runtime_s', 'goal_x', 'goal_y', 'max_vel'});
end

function write_report(report_path, detail_tbl, summary_tbl, benchmark_mode, selected_map_label, view_mode, timeout_steps, stop_mode, stop_requested, stop_reason, stop_map_name, stop_repeat_id)
f = fopen(report_path, 'w');
if strcmpi(benchmark_mode, 'test')
    fprintf(f, '# Project - Benchmark jedne mapy\n\n');
else
    fprintf(f, '# Project - Benchmark vsech map\n\n');
end
fprintf(f, 'Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.\n\n');
fprintf(f, 'Pouzite nastaveni:\n');
fprintf(f, '- planner: `A*`\n');
fprintf(f, '- smoothing: `chaikin`\n');
fprintf(f, '- controller: `pure pursuit`\n');
fprintf(f, '- max velocity: `1.0 m/s`\n');
fprintf(f, '- benchmark mode: `%s`\n', benchmark_mode);
fprintf(f, '- view mode: `%s`\n', string(logical(view_mode)));
fprintf(f, '- stop mode: `%s`\n', stop_mode);
if strcmpi(benchmark_mode, 'test')
    fprintf(f, '- selected test map: `%s`\n', selected_map_label);
end
repeats_logged = 0;
if ~isempty(detail_tbl) && height(detail_tbl) > 0
    repeats_logged = max(detail_tbl.repeat_id);
end
fprintf(f, '- repeats na mapu: `%d`\n', repeats_logged);
fprintf(f, '- timeout: `%d` kroku (`%.1f s` pri `Ts = 0.1 s`)\n', timeout_steps, 0.1 * timeout_steps);
fprintf(f, '- mode cases: `%s`\n', detect_case_mode(detail_tbl));
fprintf(f, '- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode\n');
fprintf(f, '- validace pripadu: clearance + existence cesty pres planner\n');
if strcmpi(benchmark_mode, 'test')
    fprintf(f, '- cases file: `project_test_all_maps_cases_test_%s.csv`\n\n', selected_map_label);
else
    fprintf(f, '- cases file: `project_test_all_maps_cases_full.csv`\n\n');
end
if stop_requested
    fprintf(f, '- benchmark byl ukoncen predcasne na prvnim failu: `%s`, mapa `%s`, repeat `%d`\n\n', stop_reason, stop_map_name, stop_repeat_id);
end

fprintf(f, '## Konfigurace behu\n\n');
fprintf(f, '| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |\n');
fprintf(f, '|---|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(summary_tbl)
    idx = find(strcmp(detail_tbl.map_name, summary_tbl.map_name{i}), 1, 'first');
    theta_txt = theta_to_pi_text(detail_tbl.start_theta(idx));
    fprintf(f, '| %s | %.3f | %.3f | %.3f (%s) | %.3f | %.3f | %.1f |\n', ...
        summary_tbl.map_name{i}, detail_tbl.start_x(idx), detail_tbl.start_y(idx), ...
        detail_tbl.start_theta(idx), theta_txt, summary_tbl.goal_x(i), summary_tbl.goal_y(i), summary_tbl.max_vel(i));
end

fprintf(f, '## Souhrn\n\n');
fprintf(f, '| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |\n');
fprintf(f, '|---|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(summary_tbl)
    fprintf(f, '| %s | %.2f | %d | %d | %d | %d | %.1f | %.3f | %.3f |\n', ...
        summary_tbl.map_name{i}, summary_tbl.success_rate(i), summary_tbl.goal_runs(i), ...
        summary_tbl.wall_runs(i), summary_tbl.out_runs(i), summary_tbl.timeout_runs(i), ...
        summary_tbl.mean_steps_success(i), summary_tbl.mean_goal_error_m(i), summary_tbl.mean_runtime_s(i));
end

fprintf(f, '\n## Detailni behy\n\n');
fprintf(f, '| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |\n');
fprintf(f, '|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|\n');
for i = 1:height(detail_tbl)
    theta_txt = theta_to_pi_text(detail_tbl.start_theta(i));
    fprintf(f, '| %s | %d | %.3f | %.3f | %.3f (%s) | %.3f | %.3f | %s | %d | %d | %.3f | %.3f | %.3f |\n', ...
        detail_tbl.map_name{i}, detail_tbl.repeat_id(i), detail_tbl.start_x(i), detail_tbl.start_y(i), ...
        detail_tbl.start_theta(i), theta_txt, detail_tbl.goal_x(i), detail_tbl.goal_y(i), detail_tbl.status(i), ...
        detail_tbl.success(i), detail_tbl.steps(i), detail_tbl.goal_error_m(i), ...
        detail_tbl.travelled_m(i), detail_tbl.runtime_s(i));
end
fclose(f);
end

function p = rel_path(path_abs, root_abs)
p = strrep(path_abs, [root_abs filesep], '');
end

function txt = theta_to_pi_text(theta)
ratio = theta / pi;
candidates = [...
    -2, -3/2, -4/3, -5/4, -1, -3/4, -2/3, -1/2, -1/3, -1/4, ...
     0, ...
     1/4, 1/3, 1/2, 2/3, 3/4, 1, 5/4, 4/3, 3/2, 2];
labels = {...
    '-2*pi', '-3*pi/2', '-4*pi/3', '-5*pi/4', '-pi', '-3*pi/4', '-2*pi/3', '-pi/2', '-pi/3', '-pi/4', ...
    '0', ...
    'pi/4', 'pi/3', 'pi/2', '2*pi/3', '3*pi/4', 'pi', '5*pi/4', '4*pi/3', '3*pi/2', '2*pi'};

[err, idx] = min(abs(ratio - candidates));
if err < 0.03
    txt = labels{idx};
else
    txt = sprintf('%.3f*pi', ratio);
end
end

function cases_tbl = load_or_create_cases(map_names, maps_dir, num_repeats, case_mode, cases_file)
must_regenerate = strcmpi(case_mode, 'regenerate') || ~isfile(cases_file);

if must_regenerate
    rows = cell(numel(map_names) * num_repeats, 8);
    row_idx = 1;
    for m = 1:numel(map_names)
        map_path = fullfile(maps_dir, map_names{m});
        map_base = load_map(map_path);
        map_base.discretization_step = 0.2;
        map_base.goal_tolerance = 0.5;
        discrete_map_base = generate_discrete_map(map_base);

        case_rows = generate_map_cases(map_base, discrete_map_base, erase(map_names{m}, '.txt'), num_repeats, m);
        for r = 1:num_repeats
            start_pose = [case_rows{r, 3}, case_rows{r, 4}, case_rows{r, 5}];
            goal_xy = [case_rows{r, 6}, case_rows{r, 7}];
            rows(row_idx, :) = { ...
                case_rows{r, 1}, ...
                case_rows{r, 2}, ...
                start_pose(1), ...
                start_pose(2), ...
                start_pose(3), ...
                goal_xy(1), ...
                goal_xy(2), ...
                string(lower(case_mode))};
            row_idx = row_idx + 1;
        end
    end

    cases_tbl = cell2table(rows, 'VariableNames', ...
        {'map_name', 'repeat_id', 'start_x', 'start_y', 'start_theta', 'goal_x', 'goal_y', 'cases_mode'});
    writetable(cases_tbl, cases_file);
    return;
end

cases_tbl = readtable(cases_file, 'TextType', 'string');
required = {'map_name', 'repeat_id', 'start_x', 'start_y', 'start_theta', 'goal_x', 'goal_y'};
for i = 1:numel(required)
    if ~ismember(required{i}, cases_tbl.Properties.VariableNames)
        error('Cases file is missing required column "%s".', required{i});
    end
end

expected_rows = numel(map_names) * num_repeats;
if height(cases_tbl) ~= expected_rows
    warning('Cases file row count (%d) does not match expected count (%d). Regenerating cases file.', height(cases_tbl), expected_rows);
    cases_tbl = load_or_create_cases(map_names, maps_dir, num_repeats, 'regenerate', cases_file);
    return;
end
cases_tbl.cases_mode = repmat(string(lower(case_mode)), height(cases_tbl), 1);
end

function case_rows = generate_map_cases(map_base, discrete_map_base, map_name, num_repeats, map_idx)
candidate_clearance_m = 0.55;
goal_clearance_m = 0.65;
pts = collect_candidate_points(discrete_map_base, candidate_clearance_m);
if size(pts, 1) < max(20, 2 * num_repeats)
    pts = collect_candidate_points(discrete_map_base, 0.40);
end
if size(pts, 1) < 4
    error('Map %s: not enough safe candidate points.', map_name);
end

map_span = hypot(map_base.limits(3) - map_base.limits(1), map_base.limits(4) - map_base.limits(2));
min_pair_dist_m = max(3.0, 0.35 * map_span);

num_seed = min(size(pts, 1), max(12, ceil(1.5 * num_repeats)));
start_seed_idx = farthest_point_subset(pts, num_seed, 100 + map_idx);
goal_seed_idx = farthest_point_subset(flipud(pts), num_seed, 400 + map_idx);
start_pool = pts(start_seed_idx, :);
goal_pool = flipud(pts);
goal_pool = goal_pool(goal_seed_idx, :);

candidate_rows = {};
candidate_scores = [];
candidate_starts = zeros(0, 2);
candidate_goals = zeros(0, 2);

[ii, jj] = ndgrid(1:size(start_pool, 1), 1:size(goal_pool, 1));
pair_idx = [ii(:), jj(:)];
rng(700 + map_idx, 'twister');
pair_idx = pair_idx(randperm(size(pair_idx, 1)), :);
max_pair_evals = min(size(pair_idx, 1), 180);
good_target = max(40, 4 * num_repeats);

for p = 1:max_pair_evals
    i = pair_idx(p, 1);
    j = pair_idx(p, 2);
    start_xy = start_pool(i, :);
    goal_xy = goal_pool(j, :);
        if norm(start_xy - goal_xy) < min_pair_dist_m
            continue;
        end
        if point_clearance_to_walls(goal_xy, map_base.walls) < goal_clearance_m
            continue;
        end

        start_theta = -pi + 2 * pi * rand();
        map_case = map_base;
        map_case.goal = goal_xy;
        [ok, metrics] = path_exists(map_case, discrete_map_base, [start_xy, start_theta], 0.2);
        if ~ok
            continue;
        end

        straight_dist = norm(goal_xy - start_xy);
        winding_ratio = metrics.path_length_m / max(straight_dist, 1e-6);
        if winding_ratio < 1.10 && metrics.path_length_m < 0.5 * map_span
            continue;
        end

        score = metrics.path_length_m + 0.7 * metrics.heading_variation_rad + 0.5 * metrics.turn_count;
        candidate_rows(end + 1, :) = {map_name, 0, start_xy(1), start_xy(2), start_theta, goal_xy(1), goal_xy(2), "regenerate"}; %#ok<AGROW>
        candidate_scores(end + 1, 1) = score; %#ok<AGROW>
        candidate_starts(end + 1, :) = start_xy; %#ok<AGROW>
        candidate_goals(end + 1, :) = goal_xy; %#ok<AGROW>
        if size(candidate_starts, 1) >= good_target
            break;
        end
end

if isempty(candidate_rows)
    error('Map %s: no challenging valid benchmark pairs found.', map_name);
end

[~, order] = sort(candidate_scores, 'descend');
case_rows = cell(num_repeats, 8);
picked = 0;
picked_starts = zeros(0, 2);
picked_goals = zeros(0, 2);
for k = 1:numel(order)
    idx = order(k);
    start_xy = candidate_starts(idx, :);
    goal_xy = candidate_goals(idx, :);
    if ~isempty(picked_starts)
        if any(vecnorm(picked_starts - start_xy, 2, 2) < 1.5) || any(vecnorm(picked_goals - goal_xy, 2, 2) < 1.5)
            continue;
        end
    end

    picked = picked + 1;
    row = candidate_rows(idx, :);
    row{2} = picked;
    case_rows(picked, :) = row;
    picked_starts(end + 1, :) = start_xy; %#ok<AGROW>
    picked_goals(end + 1, :) = goal_xy; %#ok<AGROW>
    if picked >= num_repeats
        break;
    end
end

if picked < num_repeats
    for k = 1:numel(order)
        idx = order(k);
        row = candidate_rows(idx, :);
        already = false;
        for c = 1:picked
            if isequal(case_rows(c, 3:7), row(3:7))
                already = true;
                break;
            end
        end
        if already
            continue;
        end
        picked = picked + 1;
        row{2} = picked;
        case_rows(picked, :) = row;
        if picked >= num_repeats
            break;
        end
    end
end
end

function pts = collect_candidate_points(discrete_map_base, clearance_m)
grid = discrete_map_base.map;
[ny, nx] = size(grid);
limits = discrete_map_base.limits;
margin = clearance_cells(limits, [nx ny], clearance_m);

valid = false(size(grid));
for y = 1 + margin:ny - margin
    for x = 1 + margin:nx - margin
        if grid(y, x) ~= 0
            continue;
        end
        if all(grid(y-margin:y+margin, x-margin:x+margin) == 0, 'all')
            valid(y, x) = true;
        end
    end
end

[ys, xs] = find(valid);
pts = zeros(numel(xs), 2);
for i = 1:numel(xs)
    pts(i, :) = discrete_to_world_local(xs(i), ys(i), limits, [nx ny]);
end
end

function idx = farthest_point_subset(pts, k, seed)
rng(seed, 'twister');
n = size(pts, 1);
if k >= n
    idx = (1:n)';
    return;
end

idx = zeros(k, 1);
idx(1) = randi(n);
dist_best = inf(n, 1);
for t = 2:k
    last_pt = pts(idx(t - 1), :);
    dist_best = min(dist_best, vecnorm(pts - last_pt, 2, 2));
    [~, idx(t)] = max(dist_best);
end
idx = unique(idx, 'stable');
end

function c = point_clearance_to_walls(pt, walls)
if isempty(walls)
    c = inf;
    return;
end
c = inf;
for i = 1:size(walls, 1)
    c = min(c, point_segment_distance(pt, walls(i, 1:2), walls(i, 3:4)));
end
end

function d = point_segment_distance(p, a, b)
ab = b - a;
den = dot(ab, ab);
if den <= 1e-12
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / den;
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end

function [map_case, discrete_map_case, start_pose] = apply_case(map_base, discrete_map_base, run_case)
map_case = map_base;
map_case.goal = [run_case.goal_x(1), run_case.goal_y(1)];
discrete_map_case = discrete_map_base;
start_pose = [run_case.start_x(1), run_case.start_y(1), run_case.start_theta(1)];
end

function seed = case_seed_from_row(run_case)
vals = [round(1000 * run_case.start_x(1)), ...
    round(1000 * run_case.start_y(1)), ...
    round(1000 * run_case.start_theta(1)), ...
    round(1000 * run_case.goal_x(1)), ...
    round(1000 * run_case.goal_y(1))];
seed = uint32(2166136261);
for i = 1:numel(vals)
    seed = bitxor(seed, uint32(typecast(int32(vals(i)), 'uint32')));
    seed = uint32(mod(uint64(seed) * 16777619, 2^32));
end
seed = double(mod(seed, intmax('uint32') - 1)) + 1;
end

function mode_txt = detect_case_mode(detail_tbl)
mode_txt = 'reuse';
if ~isempty(detail_tbl) && height(detail_tbl) > 0 && ismember('cases_mode', detail_tbl.Properties.VariableNames)
    mode_txt = char(detail_tbl.cases_mode(1));
end
end

function [ok, metrics] = path_exists(map_case, discrete_map_case, start_pose, clearance_m)
ok = false;
metrics = struct('path_length_m', inf, 'heading_variation_rad', inf, 'turn_count', inf);
read_only_vars = struct();
read_only_vars.map = map_case;
read_only_vars.discrete_map = discrete_map_case;

public_vars = struct();
public_vars.path = [];
public_vars.replan_path = true;
public_vars.force_grid_planner = true;
public_vars.path_planner_mode = 'astar';
public_vars.path_smoothing_mode = 'chaikin';
public_vars.path_clearance_m = clearance_m;
public_vars.path_resample_ds_m = 0.10;
public_vars.path_refine_iters = 2;
public_vars.smooth_chaikin_iters = 4;
public_vars.estimated_pose = start_pose;

path = plan_path(read_only_vars, public_vars);
ok = ~isempty(path) && size(path, 1) >= 2 ...
    && all(isfinite(path(:))) ...
    && norm(path(1, :) - start_pose(1:2)) < 0.6 ...
    && norm(path(end, :) - map_case.goal(1:2)) < 0.6;
if ok
    dxy = diff(path, 1, 1);
    seg_len = vecnorm(dxy, 2, 2);
    headings = atan2(dxy(:, 2), dxy(:, 1));
    dhead = abs(wrap_to_pi_local(diff(headings)));
    metrics.path_length_m = sum(seg_len);
    metrics.heading_variation_rad = sum(dhead);
    metrics.turn_count = sum(dhead > 20 * pi / 180);
end
end

function margin = clearance_cells(limits, dims, clearance_m)
nx = dims(1);
ny = dims(2);
if nx <= 1 || ny <= 1
    margin = 1;
    return;
end
dx = (limits(3) - limits(1)) / (nx - 1);
dy = (limits(4) - limits(2)) / (ny - 1);
cell_m = max(dx, dy);
margin = max(1, ceil(clearance_m / max(cell_m, 1e-6)));
end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2 * pi) - pi;
end

function save_run_review_figure(history_dir, map_name, repeat_id, map, start_pose, result)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 850]);
hold on;
axis equal;
grid on;

for i = 1:size(map.walls, 1)
    line([map.walls(i, 1), map.walls(i, 3)], [map.walls(i, 2), map.walls(i, 4)], ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 5, 'HandleVisibility', 'off');
end
line([map.limits(1), map.limits(3)], [map.limits(2), map.limits(2)], ...
    'Color', 'black', 'LineWidth', 1, 'LineStyle', '--', 'HandleVisibility', 'off');
line([map.limits(3), map.limits(3)], [map.limits(2), map.limits(4)], ...
    'Color', 'black', 'LineWidth', 1, 'LineStyle', '--', 'HandleVisibility', 'off');
line([map.limits(1), map.limits(3)], [map.limits(4), map.limits(4)], ...
    'Color', 'black', 'LineWidth', 1, 'LineStyle', '--', 'HandleVisibility', 'off');
line([map.limits(1), map.limits(1)], [map.limits(2), map.limits(4)], ...
    'Color', 'black', 'LineWidth', 1, 'LineStyle', '--', 'HandleVisibility', 'off');

if isfield(map, 'gnss_denied') && ~isempty(map.gnss_denied)
    for i = 1:size(map.gnss_denied, 1)
        pgon = polyshape(map.gnss_denied(i, 1:2:end), map.gnss_denied(i, 2:2:end));
        plot(pgon, 'FaceColor', 'magenta', 'FaceAlpha', 0.08, ...
            'EdgeColor', 'magenta', 'LineStyle', '--', 'HandleVisibility', 'off');
    end
end

if ~isempty(result.path)
    plot(result.path(:, 1), result.path(:, 2), '-', 'Color', [0.10 0.80 0.20], ...
        'LineWidth', 2.8, 'DisplayName', 'Planned path');
end

if ~isempty(result.pose_history)
    plot(result.pose_history(:, 1), result.pose_history(:, 2), '-', ...
        'Color', [0.10 0.80 0.90], 'LineWidth', 2.4, 'DisplayName', 'True trajectory');
end

plot(start_pose(1), start_pose(2), 'kp', 'MarkerSize', 14, 'LineWidth', 2, 'DisplayName', 'Start');
quiver(start_pose(1), start_pose(2), 0.55 * cos(start_pose(3)), 0.55 * sin(start_pose(3)), ...
    0, 'Color', [0.05 0.05 0.05], 'LineWidth', 2, 'MaxHeadSize', 1.4, 'DisplayName', 'Start heading');
plot(map.goal(1), map.goal(2), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Goal');
plot(result.final_pose(1), result.final_pose(2), 'rx', 'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', 'Final pose');

theta_end = result.final_pose(3);
quiver(result.final_pose(1), result.final_pose(2), 0.45 * cos(theta_end), 0.45 * sin(theta_end), ...
    0, 'Color', [0.85 0.10 0.10], 'LineWidth', 2, 'MaxHeadSize', 1.5, 'HandleVisibility', 'off');

axis([map.limits(1) - 0.5, map.limits(3) + 0.5, map.limits(2) - 0.5, map.limits(4) + 0.5]);
xlabel('x [m]');
ylabel('y [m]');
title(sprintf('%s | run %02d | %s | steps=%d | goal err=%.3f m', ...
    map_name, repeat_id, result.status, result.steps, result.goal_error_m), ...
    'Interpreter', 'none', 'FontWeight', 'bold');
legend('Location', 'eastoutside');

base_name = sprintf('%s_run_%02d_%s', map_name, repeat_id, char(result.status));
png_path = fullfile(history_dir, [base_name '.png']);
fig_path = fullfile(history_dir, [base_name '.fig']);
exportgraphics(fig, png_path, 'BackgroundColor', 'white', 'Resolution', 170);
savefig(fig, fig_path);
close(fig);
end

function live_view = init_live_view(map, agent_pose)
fig = figure('Name', 'Project benchmark live view', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [80 80 1300 800]);
clf(fig);
ax = axes(fig, 'Position', [0.07 0.10 0.68 0.82]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
axis(ax, [map.limits(1)-1 map.limits(3)+1 map.limits(2)-1 map.limits(4)+1]);
title(ax, 'Project benchmark live view', 'FontWeight', 'bold');

[cx, cy] = local_create_circle(map.goal(1), map.goal(2), map.goal_tolerance);
plot(ax, cx, cy, 'Color', 'green', 'LineWidth', 2, 'HandleVisibility', 'off');
[sx, sy] = local_create_arrow(agent_pose(1:2), agent_pose(3), 0.55);
plot(ax, sx, sy, 'Color', [0.10 0.10 0.10], 'LineWidth', 2.2);
[axp, ayp] = local_create_arrow(agent_pose(1:2), agent_pose(3), 0.5);
h_agent = plot(ax, axp, ayp, 'Color', 'blue', 'LineWidth', 2);
h_path = plot(ax, nan, nan, '-', 'Color', [0.10 0.80 0.20], 'LineWidth', 2.6);
h_est = plot(ax, nan, nan, 'r-', 'LineWidth', 1.8);
h_traj = plot(ax, nan, nan, '-', 'Color', [0.10 0.80 0.90], 'LineWidth', 2.0);

for i = 1:size(map.walls, 1)
    xs = [map.walls(i,1), map.walls(i,3)];
    ys = [map.walls(i,2), map.walls(i,4)];
    line(ax, xs, ys, 'Color', 'black', 'LineWidth', 5);
end
line(ax, [map.limits(1), map.limits(3)], [map.limits(2), map.limits(2)], 'Color', 'black', 'LineWidth', 1, 'LineStyle', '--');
line(ax, [map.limits(3), map.limits(3)], [map.limits(2), map.limits(4)], 'Color', 'black', 'LineWidth', 1, 'LineStyle', '--');
line(ax, [map.limits(1), map.limits(3)], [map.limits(4), map.limits(4)], 'Color', 'black', 'LineWidth', 1, 'LineStyle', '--');
line(ax, [map.limits(1), map.limits(1)], [map.limits(2), map.limits(4)], 'Color', 'black', 'LineWidth', 1, 'LineStyle', '--');
for i = 1:size(map.gnss_denied, 1)
    pgon = polyshape(map.gnss_denied(i, 1:2:end), map.gnss_denied(i, 2:2:end));
    plot(ax, pgon, 'FaceColor', 'magenta', 'FaceAlpha', 0.1, 'EdgeColor', 'magenta', 'LineStyle', '--');
end

panel = uipanel(fig, 'Title', 'Run Diagnostics', 'FontWeight', 'bold', ...
    'BackgroundColor', 'white', 'Position', [0.76 0.10 0.22 0.80]);
status_text = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05 0.04 0.90 0.92], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'white', ...
    'FontName', 'Consolas', 'FontSize', 12, 'String', '');

live_view = struct('figure', fig, 'axes', ax, 'agent_handle', h_agent, ...
    'path_handle', h_path, 'estimate_handle', h_est, 'traj_handle', h_traj, ...
    'status_text', status_text, 'panel', panel);
drawnow;
end

function live_view = update_live_view(live_view, private_vars, read_only_vars, public_vars)
if ~isgraphics(live_view.figure)
    return;
end
if isgraphics(live_view.agent_handle)
    [axp, ayp] = local_create_arrow(private_vars.agent_pose(1:2), private_vars.agent_pose(3), 0.5);
    set(live_view.agent_handle, 'XData', axp, 'YData', ayp);
end
if isgraphics(live_view.path_handle) && isfield(public_vars, 'path') && ~isempty(public_vars.path)
    set(live_view.path_handle, 'XData', public_vars.path(:,1), 'YData', public_vars.path(:,2));
end
if isgraphics(live_view.estimate_handle) && isfield(read_only_vars, 'est_position_history') ...
        && size(read_only_vars.est_position_history, 1) > 1
    est = read_only_vars.est_position_history;
    est = est(all(isfinite(est(:,1:2)), 2), :);
    if ~isempty(est)
        set(live_view.estimate_handle, 'XData', est(:,1), 'YData', est(:,2));
    end
end
if isgraphics(live_view.traj_handle) && ~isempty(private_vars.agent_position_history)
    set(live_view.traj_handle, 'XData', private_vars.agent_position_history(:,1), ...
        'YData', private_vars.agent_position_history(:,2));
end
if isgraphics(live_view.status_text)
    [v_cmd, w_cmd] = motion_vector_to_vw(public_vars.motion_vector, read_only_vars.agent_drive.interwheel_dist);
    nav_state = "unknown";
    if isfield(public_vars, 'nav_state') && ~isempty(public_vars.nav_state)
        nav_state = string(public_vars.nav_state);
    end
    reason = "";
    if isfield(public_vars, 'localization_quality') && isfield(public_vars.localization_quality, 'reason')
        reason = string(public_vars.localization_quality.reason);
    end
    q = get_field_or_local(public_vars, 'localization_quality', struct());
    pf_cluster = get_field_or_local(q, 'pf_cluster_radius_m', nan);
    pf_mass = get_field_or_local(q, 'pf_dominant_mass', nan);
    pf_ratio = get_field_or_local(q, 'pf_top_ratio', nan);
    pf_hypothesis_count = get_field_or_local(q, 'pf_hypothesis_count', nan);
    scan_match_cost = get_field_or_local(q, 'scan_match_cost', nan);
    h1_mass = get_field_or_local(q, 'h1_mass', nan);
    h1_score = get_field_or_local(q, 'h1_scan_score', nan);
    h2_mass = get_field_or_local(q, 'h2_mass', nan);
    h2_score = get_field_or_local(q, 'h2_scan_score', nan);
    h3_mass = get_field_or_local(q, 'h3_mass', nan);
    h3_score = get_field_or_local(q, 'h3_scan_score', nan);
    commit_watchdog_counter = get_field_or_local(q, 'commit_watchdog_counter', nan);
    reseed_success_count = get_field_or_local(q, 'reseed_success_count', nan);
    reseed_attempt_count = get_field_or_local(q, 'reseed_attempt_count', nan);
    path_quality_score = get_field_or_local(get_field_or_local(public_vars, 'path_quality', struct()), 'score', nan);
    commit = get_field_or_local(get_field_or_local(public_vars, 'localization_commit', struct()), 'state', "search");
    live_view.status_text.String = sprintf( ...
        ['%-14s %5d\n%-14s %s\n%-14s %s\n%-14s %s\n\n' ...
         '%-14s %7.3f m/s\n%-14s %7.3f rad/s\n\n' ...
         '%-14s %7.3f m\n%-14s %7.3f\n%-14s %7.3f\n%-14s %4.0f\n%-14s %7.3f\n' ...
         '%-14s %4.2f/%4.2f\n%-14s %4.2f/%4.2f\n%-14s %4.2f/%4.2f\n' ...
         '%-14s %3.0f\n%-14s %3.0f / %3.0f\n%-14s %7.3f'], ...
        'step:', read_only_vars.counter, ...
        'state:', nav_state, ...
        'commit:', string(commit), ...
        'reason:', string(reason), ...
        'v:', v_cmd, ...
        'w:', w_cmd, ...
        'pf cluster:', pf_cluster, ...
        'pf mass:', pf_mass, ...
        'pf ratio:', pf_ratio, ...
        'hypotheses:', pf_hypothesis_count, ...
        'scan cost:', scan_match_cost, ...
        'H1 m/s:', h1_mass, h1_score, ...
        'H2 m/s:', h2_mass, h2_score, ...
        'H3 m/s:', h3_mass, h3_score, ...
        'watchdog:', commit_watchdog_counter, ...
        'reseeds:', reseed_success_count, reseed_attempt_count, ...
        'path quality:', path_quality_score);
end
drawnow limitrate;
end

function value = get_field_or_local(s, field_name, fallback)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = fallback;
end
end

function [v_cmd, w_cmd] = motion_vector_to_vw(motion_vector, wheelbase)
if isempty(motion_vector) || numel(motion_vector) ~= 2 || any(~isfinite(motion_vector))
    v_cmd = 0;
    w_cmd = 0;
    return;
end
vR = motion_vector(1);
vL = motion_vector(2);
v_cmd = 0.5 * (vR + vL);
w_cmd = (vR - vL) / max(wheelbase, 1e-6);
end

function [x, y] = local_create_circle(cx, cy, r)
t = linspace(0, 2 * pi, 100);
x = cx + r * cos(t);
y = cy + r * sin(t);
end

function [x, y] = local_create_arrow(p, theta, len)
tip = p(:)' + len * [cos(theta), sin(theta)];
left = p(:)' + 0.35 * len * [cos(theta + 2.5), sin(theta + 2.5)];
right = p(:)' + 0.35 * len * [cos(theta - 2.5), sin(theta - 2.5)];
x = [p(1), tip(1), left(1), tip(1), right(1)];
y = [p(2), tip(2), left(2), tip(2), right(2)];
end
