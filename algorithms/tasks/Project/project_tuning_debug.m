%% Project tuning debug - single deterministic run
warm_mode_env = getenv('PROJECT_DEBUG_WARM');
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
sessions_root = fullfile(out_dir, 'RunSessions');

% Debug configuration
map_label = getenv('PROJECT_DEBUG_MAP');
if isempty(map_label)
    map_label = 'indoor_2';
end
map_list = split_env_list(getenv('PROJECT_DEBUG_MAP_LIST'));
if isempty(map_list)
    map_list = string(map_label);
end
repeat_id = str2double(getenv('PROJECT_DEBUG_REPEAT'));
if ~isfinite(repeat_id) || repeat_id < 1
    repeat_id = 1;
end
repeat_list = parse_numeric_list(getenv('PROJECT_DEBUG_REPEAT_LIST'));
if isempty(repeat_list)
    repeat_list = repeat_id;
end
view_mode_env = getenv('PROJECT_DEBUG_VIEW');
if isempty(view_mode_env)
    view_mode = true;
else
    view_mode = any(strcmpi(strtrim(view_mode_env), {'1', 'true', 'on', 'yes'}));
end
timeout_steps = 2050;
timeout_steps_env = str2double(getenv('PROJECT_DEBUG_TIMEOUT'));
if isfinite(timeout_steps_env) && timeout_steps_env >= 50
    timeout_steps = round(timeout_steps_env);
end
controller_mode = getenv('PROJECT_CONTROLLER_MODE');
if isempty(controller_mode)
    controller_mode = 'pure_pursuit';
end
path_smoothing_mode = getenv('PROJECT_SMOOTHING_MODE');
if isempty(path_smoothing_mode)
    path_smoothing_mode = 'chaikin';
end
localization_mode = getenv('PROJECT_LOCALIZATION_MODE');
if isempty(localization_mode)
    localization_mode = 'fusion';
end
save_outputs_env = getenv('PROJECT_DEBUG_SAVE_OUTPUTS');
if isempty(save_outputs_env)
    save_outputs = true;
else
    save_outputs = any(strcmpi(strtrim(save_outputs_env), {'1', 'true', 'on', 'yes'}));
end
shared_live_view_env = getenv('PROJECT_SHARED_LIVE_VIEW');
shared_view_mode = parse_shared_view_mode(getenv('PROJECT_SHARED_VIEW_MODE'));
concurrent_runs = 1;
concurrent_runs_env = getenv('PROJECT_DEBUG_CONCURRENT_RUNS');
if ~isempty(concurrent_runs_env)
    concurrent_runs_val = str2double(concurrent_runs_env);
    if isfinite(concurrent_runs_val) && concurrent_runs_val >= 1
        concurrent_runs = max(1, round(concurrent_runs_val));
    end
end
use_process_parallel = false;
if concurrent_runs > 1 && ~supports_parallel_runs()
    if supports_process_parallel_runs()
        warning('Parallel Computing Toolbox not available. Using process-based debug parallelism via MATLAB R2023b workers.');
        use_process_parallel = true;
    else
        warning('Parallel Computing Toolbox not available and no MATLAB R2023b worker executable found. Falling back to serial debug execution.');
        concurrent_runs = 1;
    end
end
session_label = getenv('PROJECT_SESSION_LABEL');
session_dir = create_debug_session_dir(sessions_root, 'debug', session_label);
cases_file = getenv('PROJECT_CASES_FILE');
if isempty(cases_file)
    if numel(map_list) == 1
        cases_file = fullfile(out_dir, sprintf('project_test_all_maps_cases_test_%s.csv', char(map_list(1))));
    else
        cases_file = fullfile(out_dir, 'project_test_all_maps_cases_full.csv');
    end
end

if ~exist(cases_file, 'file')
    error('Cases file not found: %s. Run project_test_all_maps.m first.', cases_file);
end

cases_tbl = readtable(cases_file);
case_mask = ismember(string(cases_tbl.map_name), map_list) & ismember(cases_tbl.repeat_id, repeat_list);
selected_cases = cases_tbl(case_mask, :);
if isempty(selected_cases) || height(selected_cases) == 0
    error('No cases found for requested debug selection.');
end
selected_cases = sortrows(selected_cases, {'map_name', 'repeat_id'});
debug_tasks = build_debug_tasks(selected_cases);
shared_live_view = view_mode && numel(debug_tasks) > 1 && numel(unique(string(selected_cases.map_name), 'stable')) == 1;
if ~isempty(shared_live_view_env)
    shared_live_view = shared_live_view && any(strcmpi(strtrim(shared_live_view_env), {'1', 'true', 'on', 'yes'}));
end
multi_run_view = view_mode && numel(debug_tasks) > 1 && ~shared_live_view;
dashboard = [];
if multi_run_view
    dashboard = init_debug_batch_dashboard(debug_tasks, timeout_steps, concurrent_runs);
end

results_summary = cell(numel(debug_tasks), 8);
task_results = cell(numel(debug_tasks), 1);
if numel(debug_tasks) == 1 || concurrent_runs <= 1
    for i = 1:numel(debug_tasks)
        worker_view = view_mode && numel(debug_tasks) == 1;
        task_result = execute_debug_case(debug_tasks(i), maps_dir, session_dir, timeout_steps, worker_view, controller_mode, path_smoothing_mode, localization_mode, save_outputs);
        results_summary(i, :) = debug_summary_row(task_result);
        task_results{i} = task_result;
        if multi_run_view
            dashboard = update_debug_batch_dashboard(dashboard, i, task_result);
        end
    end
elseif use_process_parallel
    [task_results, results_summary, dashboard] = execute_debug_cases_process_parallel( ...
        debug_tasks, maps_dir, session_dir, timeout_steps, controller_mode, ...
        path_smoothing_mode, localization_mode, save_outputs, concurrent_runs, ...
        dashboard, multi_run_view, project_root);
else
    pool = gcp('nocreate');
    target_workers = min(concurrent_runs, numel(debug_tasks));
    if isempty(pool) || pool.NumWorkers ~= target_workers
        if ~isempty(pool)
            delete(pool);
        end
        pool = parpool('local', target_workers);
    end
    future_struct = struct('future', {}, 'task_idx', {});
    next_task_idx = 1;
    while next_task_idx <= numel(debug_tasks) || ~isempty(future_struct)
        while next_task_idx <= numel(debug_tasks) && numel(future_struct) < target_workers
            fut = parfeval(pool, @execute_debug_case, 1, debug_tasks(next_task_idx), maps_dir, session_dir, timeout_steps, false, controller_mode, path_smoothing_mode, localization_mode, save_outputs);
            future_struct(end + 1).future = fut; %#ok<AGROW>
            future_struct(end).task_idx = next_task_idx;
            if multi_run_view
                dashboard = update_debug_batch_dashboard_status(dashboard, next_task_idx, "running", "", nan, nan);
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
        results_summary(task_idx, :) = debug_summary_row(task_result);
        task_results{task_idx} = task_result;
        if multi_run_view
            dashboard = update_debug_batch_dashboard(dashboard, task_idx, task_result);
        end
    end
end

if shared_live_view
    play_shared_debug_live_view(debug_tasks, task_results, maps_dir, timeout_steps, shared_view_mode);
end

summary_tbl = cell2table(results_summary, 'VariableNames', ...
    {'map_name', 'repeat_id', 'status', 'steps', 'goal_error_m', 'travelled_m', 'runtime_s', 'session_subdir'});
writetable(summary_tbl, fullfile(session_dir, 'project_tuning_debug_runs.csv'));
fprintf('Session: %s\n', rel_path(session_dir, project_root));
fprintf('Saved: %s\n', rel_path(fullfile(session_dir, 'project_tuning_debug_runs.csv'), project_root));

function tasks = build_debug_tasks(selected_cases)
tasks = repmat(struct( ...
    'map_name', "", ...
    'repeat_id', 0, ...
    'start_pose', [nan, nan, nan], ...
    'goal_xy', [nan, nan]), height(selected_cases), 1);
for i = 1:height(selected_cases)
    tasks(i).map_name = string(selected_cases.map_name(i));
    tasks(i).repeat_id = selected_cases.repeat_id(i);
    tasks(i).start_pose = [selected_cases.start_x(i), selected_cases.start_y(i), selected_cases.start_theta(i)];
    tasks(i).goal_xy = [selected_cases.goal_x(i), selected_cases.goal_y(i)];
end
end

function task_result = execute_debug_case(task, maps_dir, session_dir, timeout_steps, view_mode, controller_mode, path_smoothing_mode, localization_mode, save_outputs)
map_path = fullfile(maps_dir, [char(task.map_name), '.txt']);
map = load_map(map_path);
map.discretization_step = 0.2;
map.goal_tolerance = 0.5;
discrete_map = generate_discrete_map(map);
map.goal = task.goal_xy;
[result, log_tbl, path_tbl] = simulate_debug_run(map, discrete_map, task.start_pose, timeout_steps, view_mode, controller_mode, path_smoothing_mode, localization_mode);
run_dir = fullfile(session_dir, sprintf('%s_run_%02d', char(task.map_name), task.repeat_id));
if ~exist(run_dir, 'dir')
    mkdir(run_dir);
end
if save_outputs
    save_debug_case_artifacts(run_dir, char(task.map_name), task.repeat_id, task.start_pose, map.goal, timeout_steps, result, log_tbl, path_tbl);
end
task_result = struct( ...
    'map_name', string(task.map_name), ...
    'repeat_id', task.repeat_id, ...
    'start_pose', task.start_pose, ...
    'goal_xy', task.goal_xy, ...
    'status', string(result.status), ...
    'steps', result.steps, ...
    'goal_error_m', result.goal_error_m, ...
    'travelled_m', result.travelled_m, ...
    'runtime_s', result.runtime_s, ...
    'run_dir', string(run_dir), ...
    'log_tbl', log_tbl, ...
    'path_tbl', path_tbl);
end

function save_debug_case_artifacts(run_dir, map_label, repeat_id, start_pose, goal_xy, timeout_steps, result, log_tbl, path_tbl)
detail_file = fullfile(run_dir, sprintf('project_tuning_debug_%s_run_%02d_log.csv', map_label, repeat_id));
writetable(log_tbl, detail_file);
paths_file = fullfile(run_dir, sprintf('project_tuning_debug_%s_run_%02d_paths.csv', map_label, repeat_id));
writetable(path_tbl, paths_file);
fig = create_debug_summary_figure(map_label, repeat_id, log_tbl);
png_file = fullfile(run_dir, sprintf('project_tuning_debug_%s_run_%02d_summary.png', map_label, repeat_id));
fig_file = fullfile(run_dir, sprintf('project_tuning_debug_%s_run_%02d_summary.fig', map_label, repeat_id));
exportgraphics(fig, png_file, 'BackgroundColor', 'white', 'Resolution', 170);
savefig(fig, fig_file);
report_file = fullfile(run_dir, sprintf('project_tuning_debug_%s_run_%02d_report.md', map_label, repeat_id));
write_debug_report(report_file, map_label, repeat_id, start_pose, goal_xy, timeout_steps, result, log_tbl);
end

function row = debug_summary_row(task_result)
row = {char(task_result.map_name), task_result.repeat_id, char(task_result.status), ...
    task_result.steps, task_result.goal_error_m, task_result.travelled_m, task_result.runtime_s, char(task_result.run_dir)};
end

function dashboard = init_debug_batch_dashboard(tasks, timeout_steps, concurrent_runs)
fig = figure('Name', 'Project tuning debug batch dashboard', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [140 140 920 560]);
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.93 0.94 0.05], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontName', 'Consolas', 'FontSize', 12, ...
    'String', sprintf('Concurrent runs: %d | Timeout: %d steps | Runs: %d', concurrent_runs, timeout_steps, numel(tasks)));
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
    'ColumnName', {'Map', 'Repeat', 'State', 'Result', 'Steps', 'GoalErr', 'RunDir'}, ...
    'ColumnWidth', {120, 60, 80, 90, 80, 90, 280});
dashboard = struct('figure', fig, 'table', tbl, 'data', {data});
drawnow;
end

function dashboard = update_debug_batch_dashboard_status(dashboard, task_idx, state_txt, result_txt, steps, goal_err)
if isempty(dashboard) || ~isgraphics(dashboard.figure)
    return;
end
dashboard.data{task_idx, 3} = char(string(state_txt));
dashboard.data{task_idx, 4} = char(string(result_txt));
dashboard.data{task_idx, 5} = steps;
dashboard.data{task_idx, 6} = goal_err;
dashboard.table.Data = dashboard.data;
drawnow limitrate;
end

function dashboard = update_debug_batch_dashboard(dashboard, task_idx, task_result)
if isempty(dashboard) || ~isgraphics(dashboard.figure)
    return;
end
dashboard.data{task_idx, 3} = 'done';
dashboard.data{task_idx, 4} = char(task_result.status);
dashboard.data{task_idx, 5} = task_result.steps;
dashboard.data{task_idx, 6} = task_result.goal_error_m;
dashboard.data{task_idx, 7} = char(task_result.run_dir);
dashboard.table.Data = dashboard.data;
drawnow limitrate;
end

function values = parse_numeric_list(raw_txt)
values = [];
if isempty(raw_txt)
    return;
end
parts = regexp(char(raw_txt), '[,; ]+', 'split');
parts = parts(~cellfun('isempty', parts));
if isempty(parts)
    return;
end
nums = str2double(parts);
nums = nums(isfinite(nums) & nums >= 1);
if isempty(nums)
    return;
end
values = unique(round(nums), 'stable');
end

function values = split_env_list(raw_txt)
values = strings(0, 1);
if isempty(raw_txt)
    return;
end
parts = regexp(char(raw_txt), '[,;]+', 'split');
parts = strtrim(parts);
parts = parts(~cellfun('isempty', parts));
if isempty(parts)
    return;
end
values = string(parts(:));
end

function session_dir = create_debug_session_dir(sessions_root, prefix, session_label)
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

function play_shared_debug_live_view(tasks, task_results, maps_dir, timeout_steps, shared_view_mode)
if isempty(task_results)
    return;
end
map_path = fullfile(maps_dir, [char(tasks(1).map_name), '.txt']);
map = load_map(map_path);
map.discretization_step = 0.2;
map.goal_tolerance = 0.5;
fig = figure('Name', 'Project tuning debug shared live view', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [80 80 1700 920]);
fig.ToolBar = 'figure';
ax = axes(fig, 'Position', [0.05 0.08 0.70 0.86]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
axis(ax, [map.limits(1)-1 map.limits(3)+1 map.limits(2)-1 map.limits(4)+1]);
title(ax, sprintf('Project tuning debug shared live view | %s | %d runs', char(tasks(1).map_name), numel(task_results)), 'FontWeight', 'bold');
for i = 1:size(map.walls, 1)
    line(ax, [map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], 'Color', 'black', 'LineWidth', 5);
end
for i = 1:size(map.gnss_denied, 1)
    pgon = polyshape(map.gnss_denied(i, 1:2:end), map.gnss_denied(i, 2:2:end));
    plot(ax, pgon, 'FaceColor', 'magenta', 'FaceAlpha', 0.08, 'EdgeColor', 'magenta', 'LineStyle', '--');
end

colors = lines(max(numel(task_results), 3));
show_extras = strcmp(shared_view_mode, 'full');
handles = repmat(struct('traj', [], 'est', [], 'path', [], 'agent', [], 'goal', [], 'label', []), numel(task_results), 1);
max_log_steps = 0;
for i = 1:numel(task_results)
    tr = task_results{i};
    plot(ax, tr.start_pose(1), tr.start_pose(2), 'o', 'Color', colors(i,:), 'MarkerSize', 6, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    [gx, gy] = local_create_circle_dbg(tr.goal_xy(1), tr.goal_xy(2), map.goal_tolerance);
    handles(i).goal = plot(ax, gx, gy, '--', 'Color', colors(i,:), 'LineWidth', 1.4, 'HandleVisibility', 'off');
    if show_extras
        final_path = extract_final_debug_path(tr.path_tbl);
        if ~isempty(final_path)
            handles(i).path = plot(ax, final_path(:,1), final_path(:,2), '--', 'Color', colors(i,:) * 0.75, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        else
            handles(i).path = plot(ax, nan, nan, '--', 'Color', colors(i,:) * 0.75, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        end
    else
        handles(i).path = plot(ax, nan, nan, '--', 'Color', colors(i,:) * 0.75, 'LineWidth', 1.2, 'Visible', 'off', 'HandleVisibility', 'off');
    end
    handles(i).traj = plot(ax, nan, nan, '-', 'Color', colors(i,:), 'LineWidth', 2.0, 'DisplayName', sprintf('run %02d true', tr.repeat_id));
    handles(i).est = plot(ax, nan, nan, ':', 'Color', colors(i,:) * 0.65, 'LineWidth', 1.2, ...
        'Visible', onoff(show_extras), 'HandleVisibility', 'off');
    handles(i).agent = plot(ax, nan, nan, '-', 'Color', colors(i,:), 'LineWidth', 2.2, 'HandleVisibility', 'off');
    handles(i).label = text(ax, tr.start_pose(1) + 0.06, tr.start_pose(2) + 0.06, sprintf('R%02d', tr.repeat_id), ...
        'Color', colors(i,:), 'FontSize', 9, 'FontWeight', 'bold', 'Clipping', 'on');
    if isfield(tr, 'log_tbl') && ~isempty(tr.log_tbl)
        max_log_steps = max(max_log_steps, height(tr.log_tbl));
    end
end
legend(ax, 'Location', 'eastoutside');

panel = uipanel(fig, 'Title', 'Shared Run Diagnostics', 'FontWeight', 'bold', ...
    'BackgroundColor', 'white', 'Position', [0.77 0.08 0.22 0.86]);
status_text = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.02 0.92 0.96], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'white', ...
    'FontName', 'Consolas', 'FontSize', 11, 'String', '');

for step_idx = 1:max_log_steps
    if ~isgraphics(fig)
        return;
    end
    lines_txt = {sprintf('%-12s %5d', 'step:', step_idx), sprintf('%-12s %5d', 'timeout:', timeout_steps), sprintf('%-12s %s', 'mode:', shared_view_mode), ' '};
    for i = 1:numel(task_results)
        tr = task_results{i};
        if ~isfield(tr, 'log_tbl') || isempty(tr.log_tbl)
            continue;
        end
        log_tbl = tr.log_tbl;
        idx = min(step_idx, height(log_tbl));
        set(handles(i).traj, 'XData', log_tbl.true_x(1:idx), 'YData', log_tbl.true_y(1:idx));
        if show_extras
            est_mask = all(isfinite([log_tbl.est_x(1:idx), log_tbl.est_y(1:idx)]), 2);
            if any(est_mask)
                set(handles(i).est, 'XData', log_tbl.est_x(est_mask), 'YData', log_tbl.est_y(est_mask), 'Visible', 'on');
            end
        end
        [axp, ayp] = local_create_arrow_dbg([log_tbl.true_x(idx), log_tbl.true_y(idx)], log_tbl.true_theta(idx), 0.42);
        set(handles(i).agent, 'XData', axp, 'YData', ayp);
        set(handles(i).label, 'Position', [log_tbl.true_x(idx) + 0.06, log_tbl.true_y(idx) + 0.06, 0]);
        if idx < height(log_tbl)
            state_txt = char(log_tbl.nav_state(idx));
            result_txt = 'running';
        else
            state_txt = char(log_tbl.nav_state(end));
            result_txt = char(tr.status);
        end
        goal_err = hypot(log_tbl.true_x(idx) - tr.goal_xy(1), log_tbl.true_y(idx) - tr.goal_xy(2));
        lines_txt{end+1} = sprintf('R%02d %-8s %-10s', tr.repeat_id, state_txt, result_txt); %#ok<AGROW>
        lines_txt{end+1} = sprintf('  xy=%5.2f,%5.2f  ge=%5.2f', log_tbl.true_x(idx), log_tbl.true_y(idx), goal_err); %#ok<AGROW>
    end
    status_text.String = sprintf('%s\n', lines_txt{:});
    drawnow limitrate;
end
end

function path_xy = extract_final_debug_path(path_tbl)
path_xy = [];
if isempty(path_tbl) || height(path_tbl) == 0
    return;
end
last_revision = max(path_tbl.path_revision);
mask = path_tbl.path_revision == last_revision;
if any(mask)
    rows = sortrows(path_tbl(mask, {'path_point_index', 'x', 'y'}), 'path_point_index');
    path_xy = [rows.x, rows.y];
end
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

function [task_results, results_summary, dashboard] = execute_debug_cases_process_parallel(tasks, maps_dir, session_dir, timeout_steps, controller_mode, path_smoothing_mode, localization_mode, save_outputs, concurrent_runs, dashboard, multi_run_view, project_root)
task_results = cell(numel(tasks), 1);
results_summary = cell(numel(tasks), 8);
worker_root = fullfile(session_dir, 'process_workers');
if ~exist(worker_root, 'dir')
    mkdir(worker_root);
end
target_workers = min(concurrent_runs, numel(tasks));
active = struct('task_idx', {}, 'result_file', {}, 'log_file', {});
next_task_idx = 1;
while next_task_idx <= numel(tasks) || ~isempty(active)
    while next_task_idx <= numel(tasks) && numel(active) < target_workers
        task_idx = next_task_idx;
        child_label = sprintf('proc_%s_r%02d_%02d_%d', char(tasks(task_idx).map_name), tasks(task_idx).repeat_id, task_idx, randi(1e6));
        child_session_dir = fullfile(fileparts(session_dir), sprintf('debug_%s', child_label));
        task_file = fullfile(worker_root, sprintf('debug_task_%02d.mat', task_idx));
        result_file = fullfile(worker_root, sprintf('debug_result_%02d.mat', task_idx));
        log_file = fullfile(worker_root, sprintf('debug_worker_%02d.log', task_idx));
        spec = struct( ...
            'project_dir', fileparts(mfilename('fullpath')), ...
            'session_label', child_label, ...
            'session_dir', child_session_dir, ...
            'cases_file', getenv('PROJECT_CASES_FILE'), ...
            'map_name', char(tasks(task_idx).map_name), ...
            'repeat_id', tasks(task_idx).repeat_id, ...
            'start_pose', tasks(task_idx).start_pose, ...
            'goal_xy', tasks(task_idx).goal_xy, ...
            'timeout_steps', timeout_steps, ...
            'controller_mode', char(controller_mode), ...
            'path_smoothing_mode', char(path_smoothing_mode), ...
            'localization_mode', char(localization_mode), ...
            'save_outputs', logical(save_outputs), ...
            'project_root', project_root, ...
            'maps_dir', maps_dir);
        save(task_file, 'spec');
        launch_matlab_worker(task_file, result_file, log_file, 'project_debug_process_worker');
        active(end + 1) = struct('task_idx', task_idx, 'result_file', result_file, 'log_file', log_file); %#ok<AGROW>
        if multi_run_view
            dashboard = update_debug_batch_dashboard_status(dashboard, task_idx, "running", "", nan, nan);
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
                task_result = failed_debug_task_result(tasks(task_idx), worker_result, active(i).log_file);
            end
            task_results{task_idx} = task_result;
            results_summary(task_idx, :) = debug_summary_row(task_result);
            if multi_run_view
                dashboard = update_debug_batch_dashboard(dashboard, task_idx, task_result);
            end
            keep_mask(i) = false;
        end
    end
    active = active(keep_mask);
end
end

function task_result = failed_debug_task_result(task, worker_result, log_file)
run_dir = "";
if isfield(worker_result, 'session_dir') && ~isempty(worker_result.session_dir)
    run_dir = string(worker_result.session_dir);
end
status_txt = "worker_error";
if isfield(worker_result, 'status') && strlength(string(worker_result.status)) > 0
    status_txt = string(worker_result.status);
end
task_result = struct( ...
    'map_name', string(task.map_name), ...
    'repeat_id', task.repeat_id, ...
    'start_pose', task.start_pose, ...
    'goal_xy', task.goal_xy, ...
    'status', status_txt, ...
    'steps', nan, ...
    'goal_error_m', inf, ...
    'travelled_m', nan, ...
    'runtime_s', nan, ...
    'run_dir', run_dir, ...
    'log_tbl', table(), ...
    'path_tbl', table());
if isfield(worker_result, 'message') && ~isempty(worker_result.message)
    warning('Debug worker failed for %s run %d: %s. Log: %s', char(task.map_name), task.repeat_id, worker_result.message, log_file);
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

function [result, log_tbl, path_tbl] = simulate_debug_run(map, discrete_map, start_pose, max_steps, view_mode, controller_mode, path_smoothing_mode, localization_mode)
tic;

% Keep debug and benchmark runs aligned: start every run with a clean
% controller state.
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
public_vars.path_smoothing_mode = path_smoothing_mode;
public_vars.path_clearance_m = 0.25;
public_vars.path_resample_ds_m = 0.10;
public_vars.path_refine_iters = 2;
public_vars.smooth_chaikin_iters = 4;
public_vars.controller_mode = controller_mode;
public_vars.localization_mode = localization_mode;

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

[map, read_only_vars, public_vars] = apply_simulation_tuning_overrides(map, read_only_vars, public_vars);

live_view = [];
if view_mode
    live_view = init_debug_live_view(map, private_vars.agent_pose);
end

function [map, read_only_vars, public_vars] = apply_simulation_tuning_overrides(map, read_only_vars, public_vars)
global PROJECT_TUNING_OVERRIDES;
if isempty(PROJECT_TUNING_OVERRIDES) || ~isstruct(PROJECT_TUNING_OVERRIDES)
    return;
end

if isfield(PROJECT_TUNING_OVERRIDES, 'map') && isstruct(PROJECT_TUNING_OVERRIDES.map)
    map = merge_debug_override_struct(map, PROJECT_TUNING_OVERRIDES.map);
end
if isfield(PROJECT_TUNING_OVERRIDES, 'read_only_vars') && isstruct(PROJECT_TUNING_OVERRIDES.read_only_vars)
    read_only_vars = merge_debug_override_struct(read_only_vars, PROJECT_TUNING_OVERRIDES.read_only_vars);
end
if isfield(PROJECT_TUNING_OVERRIDES, 'public_vars') && isstruct(PROJECT_TUNING_OVERRIDES.public_vars)
    public_vars = merge_debug_override_struct(public_vars, PROJECT_TUNING_OVERRIDES.public_vars);
end
end

function base = merge_debug_override_struct(base, override)
if isempty(override) || ~isstruct(override)
    return;
end

fields = fieldnames(override);
for i = 1:numel(fields)
    name = fields{i};
    value = override.(name);
    if isstruct(value) && isscalar(value) && isfield(base, name) && isstruct(base.(name)) && isscalar(base.(name))
        base.(name) = merge_debug_override_struct(base.(name), value);
    else
        base.(name) = value;
    end
end
end

log_rows = cell(max_steps, 85);
log_idx = 1;
path_rows = zeros(0, 5);
last_path_revision = 0;
result = struct('status', "timeout", 'success', false, 'steps', 0, ...
    'goal_error_m', norm(start_pose(1:2) - map.goal(1:2)), ...
    'travelled_m', 0, 'final_pose', start_pose, 'runtime_s', 0);

while true
    if ~all(isfinite(private_vars.agent_pose))
        result.status = "invalid_pose";
        break;
    end
    if is_in_goal(private_vars, read_only_vars)
        result.status = "goal";
        result.success = true;
        break;
    end
    try
        in_wall_now = is_in_wall(private_vars, read_only_vars);
    catch
        result.status = "wall_check_error";
        break;
    end
    if in_wall_now
        result.status = "wall";
        break;
    end
    try
        is_out_now = is_out(private_vars, read_only_vars);
    catch
        result.status = "out_check_error";
        break;
    end
    if is_out_now
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

    [v_cmd, w_cmd] = motion_vector_to_vw(public_vars.motion_vector, read_only_vars.agent_drive.interwheel_dist);
    [front_min, left_min, right_min] = lidar_sector_minima_dbg(read_only_vars);
    [path_dist, path_idx] = current_path_distance(public_vars);
    path_revision = get_field_or(public_vars, 'path_revision', 0);
    path_points = get_field_or(public_vars, 'path_num_points', 0);
    path_length = get_field_or(public_vars, 'path_length_m', 0);
    path_goal_x = nan;
    path_goal_y = nan;
    if isfield(public_vars, 'path') && ~isempty(public_vars.path)
        path_goal_x = public_vars.path(end, 1);
        path_goal_y = public_vars.path(end, 2);
    end
    dbg = get_field_or(public_vars, 'debug', struct());
    transition_reason = get_nested_or(dbg, 'last_transition_reason', "");
    track_event = get_nested_or(dbg, 'last_track_event', "");
    force_replan_reason = get_nested_or(dbg, 'force_replan_reason', "");
    force_relocalize_reason = get_nested_or(dbg, 'force_relocalize_reason', "");
    motion_dbg = get_field_or(public_vars, 'motion_debug', struct());
    speed_limiter = get_nested_or(motion_dbg, 'limiter', "none");
    base_v = get_nested_or(motion_dbg, 'base_v', nan);
    base_w = get_nested_or(motion_dbg, 'base_w', nan);
    align_active = get_nested_or(motion_dbg, 'align_active', false);
    align_target_heading = get_nested_or(motion_dbg, 'align_target_heading', nan);
    align_heading_error = get_nested_or(motion_dbg, 'align_heading_error', nan);
    ctrl_nearest_idx = get_nested_or(motion_dbg, 'nearest_idx', nan);
    ctrl_look_idx = get_nested_or(motion_dbg, 'look_idx', nan);
    ctrl_target_x = get_nested_or(motion_dbg, 'target_x', nan);
    ctrl_target_y = get_nested_or(motion_dbg, 'target_y', nan);
    ctrl_alpha = get_nested_or(motion_dbg, 'alpha', nan);
    ctrl_curvature = get_nested_or(motion_dbg, 'curvature', nan);
    ctrl_lookahead = get_nested_or(motion_dbg, 'lookahead_dist', nan);
    nav_state = get_field_or(public_vars, 'nav_state', "unknown");
    q = get_field_or(public_vars, 'localization_quality', struct());
    q_reason = get_nested_or(q, 'reason', "");
    pf_cluster = get_nested_or(q, 'pf_cluster_radius_m', nan);
    pf_mass = get_nested_or(q, 'pf_dominant_mass', nan);
    pf_ratio = get_nested_or(q, 'pf_top_ratio', nan);
    pf_hypothesis_count = get_nested_or(q, 'pf_hypothesis_count', nan);
    amb = get_field_or(public_vars, 'environment_ambiguity', struct());
    amb_active = get_nested_or(amb, 'active', false);
    amb_counter = get_nested_or(amb, 'counter', 0);
    amb_front = get_nested_or(amb, 'front', nan);
    amb_left = get_nested_or(amb, 'left', nan);
    amb_right = get_nested_or(amb, 'right', nan);
    amb_scan = get_nested_or(amb, 'scan_change', nan);
    kf_var = get_nested_or(q, 'kf_var_xy_mean', nan);
    disagree = get_nested_or(q, 'disagreement_xy_m', nan);
    track_stall = get_nested_or(q, 'track_stall_steps', nan);
    map_conflict_counter = get_nested_or(q, 'track_map_conflict_counter', nan);
    scan_mismatch_cost = get_nested_or(q, 'track_scan_mismatch_cost', nan);
    scan_mismatch_counter = get_nested_or(q, 'track_scan_mismatch_counter', nan);
    scan_match_cost = get_nested_or(q, 'scan_match_cost', nan);
    scan_match_ema = get_nested_or(q, 'scan_match_ema', nan);
    path_distance_ema = get_nested_or(q, 'path_distance_ema', nan);
    pf_cluster_ema = get_nested_or(q, 'pf_cluster_ema', nan);
    disagreement_ema = get_nested_or(q, 'disagreement_ema', nan);
    scan_match_trend = get_nested_or(q, 'scan_match_trend', nan);
    path_distance_trend = get_nested_or(q, 'path_distance_trend', nan);
    localize_stable_counter = get_nested_or(public_vars.localize, 'stable_counter', nan);
    localize_cmd_distance = get_nested_or(public_vars.localize, 'cmd_distance_accum', nan);
    localize_cmd_turn = get_nested_or(public_vars.localize, 'cmd_turn_accum', nan);
    localization_commit_state = string(get_nested_or(get_field_or(public_vars, 'localization_commit', struct()), 'state', "search"));
    ambiguity_clear_counter = get_nested_or(public_vars.ambiguity, 'clear_counter', nan);
    clear_counter_effective = get_nested_or(public_vars.environment_ambiguity, 'clear_counter', ambiguity_clear_counter);
    contradiction_counter = get_nested_or(public_vars.localize, 'contradiction_counter', nan);
    no_progress_counter = get_nested_or(public_vars.localize, 'no_progress_counter', nan);
    trace_loop_counter = get_nested_or(public_vars.localize, 'trace_loop_counter', nan);
    trace_bbox_diag = get_nested_or(public_vars.localize, 'trace_bbox_diag_m', nan);
    allow_global_reseed = get_nested_or(public_vars.localize, 'allow_global_reseed', false);
    reseed_attempt_count = get_nested_or(q, 'reseed_attempt_count', nan);
    reseed_success_count = get_nested_or(q, 'reseed_success_count', nan);
    reseed_last_score = get_nested_or(q, 'reseed_last_score', nan);
    post_reseed_hold_counter = get_nested_or(q, 'post_reseed_hold_counter', nan);
    commit_watchdog_counter = get_nested_or(q, 'commit_watchdog_counter', nan);
    hard_path_dist_counter = get_nested_or(q, 'hard_path_dist_counter', nan);
    disambiguation_goal_switches = get_nested_or(q, 'disambiguation_goal_switches', nan);
    path_quality_score = get_nested_or(get_field_or(public_vars, 'path_quality', struct()), 'score', nan);
    goal_dist_est = norm(public_vars.estimated_pose(1:2) - map.goal(1:2));
    goal_dist_true = norm(private_vars.agent_pose(1:2) - map.goal(1:2));
    loc_err = norm(public_vars.estimated_pose(1:2) - private_vars.agent_pose(1:2));
    theta_err = abs(wrap_to_pi_dbg(public_vars.estimated_pose(3) - private_vars.agent_pose(3)));
    est_jump_m = nan;
    if size(read_only_vars.est_position_history, 1) >= 2
        prev_est = read_only_vars.est_position_history(end - 1, 1:2);
        curr_est = read_only_vars.est_position_history(end, 1:2);
        if all(isfinite(prev_est)) && all(isfinite(curr_est))
            est_jump_m = norm(curr_est - prev_est);
        end
    end

    log_rows(log_idx, :) = { ...
        read_only_vars.counter, string(nav_state), string(q_reason), ...
        private_vars.agent_pose(1), private_vars.agent_pose(2), private_vars.agent_pose(3), ...
        public_vars.estimated_pose(1), public_vars.estimated_pose(2), public_vars.estimated_pose(3), ...
        loc_err, theta_err, v_cmd, w_cmd, front_min, left_min, right_min, ...
        pf_cluster, kf_var, disagree, path_dist, path_idx, track_stall, ...
        goal_dist_true, goal_dist_est, pf_mass, pf_ratio, pf_hypothesis_count, amb_active, amb_counter, ...
        amb_front, amb_left, amb_right, amb_scan, ...
        path_revision, path_points, path_length, path_goal_x, path_goal_y, ...
        string(transition_reason), string(track_event), string(force_replan_reason), string(force_relocalize_reason), ...
        string(speed_limiter), base_v, base_w, align_active, align_target_heading, align_heading_error, ...
        ctrl_nearest_idx, ctrl_look_idx, ctrl_target_x, ctrl_target_y, ctrl_alpha, ctrl_curvature, ctrl_lookahead, ...
        scan_mismatch_cost, scan_mismatch_counter, map_conflict_counter, ...
        scan_match_cost, scan_match_ema, path_distance_ema, pf_cluster_ema, disagreement_ema, scan_match_trend, path_distance_trend, ...
        localize_stable_counter, localize_cmd_distance, localize_cmd_turn, string(localization_commit_state), ambiguity_clear_counter, ...
        clear_counter_effective, contradiction_counter, no_progress_counter, trace_loop_counter, trace_bbox_diag, ...
        allow_global_reseed, reseed_attempt_count, reseed_success_count, reseed_last_score, post_reseed_hold_counter, ...
        commit_watchdog_counter, hard_path_dist_counter, disambiguation_goal_switches, path_quality_score, est_jump_m};
    log_idx = log_idx + 1;

    if path_revision > last_path_revision && isfield(public_vars, 'path') && ~isempty(public_vars.path)
        pts = public_vars.path(:, 1:2);
        rev_col = repmat(path_revision, size(pts, 1), 1);
        idx_col = (1:size(pts, 1))';
        step_col = repmat(read_only_vars.counter, size(pts, 1), 1);
        path_rows = [path_rows; [step_col, rev_col, idx_col, pts]]; %#ok<AGROW>
        last_path_revision = path_revision;
    end

    if ~isfield(public_vars, 'motion_vector') || numel(public_vars.motion_vector) ~= 2 ...
            || ~all(isfinite(public_vars.motion_vector))
        result.status = "invalid_motion";
        break;
    end

    private_vars.agent_pose = move_agent(private_vars.agent_pose, public_vars.motion_vector, ...
        read_only_vars.agent_drive, read_only_vars.sampling_period);
    if ~all(isfinite(private_vars.agent_pose))
        result.status = "invalid_pose";
        break;
    end
    private_vars.agent_position_history = [private_vars.agent_position_history; private_vars.agent_pose];

    if view_mode && ~isempty(live_view) && isgraphics(live_view.figure)
        live_view = update_debug_live_view(live_view, private_vars, read_only_vars, public_vars);
    end
    read_only_vars.counter = read_only_vars.counter + 1;
end

result.runtime_s = toc;
result.steps = max(read_only_vars.counter - 1, 0);
result.final_pose = private_vars.agent_pose;
result.goal_error_m = norm(private_vars.agent_pose(1:2) - map.goal(1:2));
if size(private_vars.agent_position_history, 1) >= 2
    result.travelled_m = sum(vecnorm(diff(private_vars.agent_position_history(:, 1:2), 1, 1), 2, 2));
end

log_rows = log_rows(1:log_idx-1, :);
log_tbl = cell2table(log_rows, 'VariableNames', ...
    {'step', 'nav_state', 'reason', ...
     'true_x', 'true_y', 'true_theta', ...
     'est_x', 'est_y', 'est_theta', ...
     'loc_err_xy_m', 'loc_err_theta_rad', 'v_cmd', 'w_cmd', ...
     'front_min', 'left_min', 'right_min', ...
     'pf_cluster_radius_m', 'kf_var_xy_mean', 'disagreement_xy_m', ...
     'path_distance_m', 'path_index', 'track_stall_steps', ...
     'goal_dist_true_m', 'goal_dist_est_m', 'pf_dominant_mass', 'pf_top_ratio', 'pf_hypothesis_count', ...
     'ambiguity_active', 'ambiguity_counter', 'ambiguity_front_m', ...
     'ambiguity_left_m', 'ambiguity_right_m', 'ambiguity_scan_change_m', ...
     'path_revision', 'path_num_points', 'path_length_m', 'path_goal_x', 'path_goal_y', ...
     'transition_reason', 'track_event', 'force_replan_reason', 'force_relocalize_reason', ...
     'speed_limiter', 'base_v_cmd', 'base_w_cmd', 'align_active', 'align_target_heading', 'align_heading_error', ...
     'ctrl_nearest_idx', 'ctrl_look_idx', 'ctrl_target_x', 'ctrl_target_y', 'ctrl_alpha', 'ctrl_curvature', 'ctrl_lookahead_dist', ...
     'scan_mismatch_cost', 'scan_mismatch_counter', 'map_conflict_counter', ...
     'scan_match_cost', 'scan_match_ema', 'path_distance_ema', 'pf_cluster_ema', 'disagreement_ema', 'scan_match_trend', 'path_distance_trend', ...
     'localize_stable_counter', 'localize_cmd_distance_m', 'localize_cmd_turn_rad', 'localization_commit_state', 'ambiguity_clear_counter', ...
     'clear_counter_effective', 'localize_contradiction_counter', 'localize_no_progress_counter', ...
     'localize_trace_loop_counter', 'localize_trace_bbox_diag_m', 'localize_allow_global_reseed', ...
     'reseed_attempt_count', 'reseed_success_count', 'reseed_last_score', 'post_reseed_hold_counter', ...
     'commit_watchdog_counter', 'hard_path_dist_counter', 'disambiguation_goal_switches', 'path_quality_score', 'est_jump_m'});

if isempty(path_rows)
    path_tbl = cell2table(cell(0, 5), 'VariableNames', ...
        {'step_created', 'path_revision', 'path_point_index', 'x', 'y'});
else
    path_tbl = array2table(path_rows, 'VariableNames', ...
        {'step_created', 'path_revision', 'path_point_index', 'x', 'y'});
end
end

function fig = create_debug_summary_figure(map_label, repeat_id, log_tbl)
fig = figure('Name', 'Project tuning debug summary', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [80 80 1400 900]);
fig.ToolBar = 'none';
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(log_tbl.step, log_tbl.loc_err_xy_m, 'b-', 'LineWidth', 1.8); hold on;
plot(log_tbl.step, log_tbl.disagreement_xy_m, 'r-', 'LineWidth', 1.6);
plot(log_tbl.step, log_tbl.path_distance_m, 'Color', [0.15 0.65 0.15], 'LineWidth', 1.6);
grid on;
xlabel('Step [-]');
ylabel('Distance [m]');
title(sprintf('%s run %02d: localization errors', map_label, repeat_id), 'FontWeight', 'bold');
legend({'XY localization error', 'PF/EKF disagreement', 'Distance to path'}, 'Location', 'best');

nexttile;
yyaxis left;
plot(log_tbl.step, log_tbl.v_cmd, '-', 'Color', [0.10 0.45 0.85], 'LineWidth', 1.8); hold on;
plot(log_tbl.step, log_tbl.w_cmd, '-', 'Color', [0.85 0.25 0.10], 'LineWidth', 1.8);
ylabel('Command');
yyaxis right;
plot(log_tbl.step, log_tbl.track_stall_steps, 'k--', 'LineWidth', 1.5);
ylabel('Track stall steps [-]');
grid on;
xlabel('Step [-]');
title('Control commands and stall counter', 'FontWeight', 'bold');
legend({'v [m/s]', 'w [rad/s]', 'stall steps'}, 'Location', 'best');

nexttile;
plot(log_tbl.step, log_tbl.front_min, 'm-', 'LineWidth', 1.6); hold on;
plot(log_tbl.step, log_tbl.left_min, 'g-', 'LineWidth', 1.6);
plot(log_tbl.step, log_tbl.right_min, 'c-', 'LineWidth', 1.6);
grid on;
xlabel('Step [-]');
ylabel('Min lidar sector distance [m]');
title('LiDAR sector minima', 'FontWeight', 'bold');
legend({'front', 'left', 'right'}, 'Location', 'best');

nexttile;
state_codes = categorical(log_tbl.nav_state);
stairs(log_tbl.step, double(state_codes), 'k-', 'LineWidth', 1.6); hold on;
plot(log_tbl.step, log_tbl.goal_dist_true_m, '-', 'Color', [0.00 0.60 0.90], 'LineWidth', 1.6);
plot(log_tbl.step, log_tbl.goal_dist_est_m, '-', 'Color', [0.90 0.15 0.15], 'LineWidth', 1.6);
grid on;
xlabel('Step [-]');
ylabel('State code / goal distance [m]');
title('State evolution and goal distances', 'FontWeight', 'bold');
legend({'state code', 'true goal dist', 'estimated goal dist'}, 'Location', 'best');
end

function write_debug_report(report_file, map_label, repeat_id, start_pose, goal_xy, timeout_steps, result, log_tbl)
f = fopen(report_file, 'w');
if f == -1
    error('Could not open %s for writing.', report_file);
end
cleanup = onCleanup(@() fclose(f));

fprintf(f, '# Project tuning debug\n\n');
fprintf(f, '- mapa: `%s`\n', map_label);
fprintf(f, '- repeat: `%d`\n', repeat_id);
fprintf(f, '- start: `[%.3f, %.3f, %.3f]`\n', start_pose(1), start_pose(2), start_pose(3));
fprintf(f, '- cil: `[%.3f, %.3f]`\n', goal_xy(1), goal_xy(2));
fprintf(f, '- timeout: `%d` kroku\n', timeout_steps);
fprintf(f, '- vysledek: `%s`\n', result.status);
fprintf(f, '- kroky: `%d`\n', result.steps);
fprintf(f, '- final goal error: `%.3f m`\n', result.goal_error_m);
fprintf(f, '- travelled: `%.3f m`\n', result.travelled_m);
fprintf(f, '- runtime: `%.3f s`\n\n', result.runtime_s);

if isempty(log_tbl) || height(log_tbl) == 0
    fprintf(f, '## Log\n\n');
    fprintf(f, '- log je prazdny; beh skoncil drive, nez se stihl zapsat prvni krok.\n');
    fclose(f);
    return;
end

fprintf(f, '## Maxima a minima\n\n');
fprintf(f, '- max XY localization error: `%.3f m`\n', max(log_tbl.loc_err_xy_m));
fprintf(f, '- max PF/EKF disagreement: `%.3f m`\n', max(log_tbl.disagreement_xy_m));
fprintf(f, '- max PF dominant mass: `%.3f`\n', max(log_tbl.pf_dominant_mass));
fprintf(f, '- max PF top ratio: `%.3f`\n', max(log_tbl.pf_top_ratio));
fprintf(f, '- max ambiguity counter: `%d`\n', round(max(log_tbl.ambiguity_counter)));
fprintf(f, '- ambiguity active steps: `%d`\n', sum(log_tbl.ambiguity_active ~= 0));
fprintf(f, '- min front lidar: `%.3f m`\n', min(log_tbl.front_min));
fprintf(f, '- max track stall steps: `%d`\n', round(max(log_tbl.track_stall_steps)));
fprintf(f, '- max scan mismatch counter: `%d`\n', round(max(log_tbl.scan_mismatch_counter)));
fprintf(f, '- max map conflict counter: `%d`\n', round(max(log_tbl.map_conflict_counter)));
fprintf(f, '- max path revision: `%d`\n', round(max(log_tbl.path_revision)));
fprintf(f, '- max estimate jump: `%.3f m`\n', max(log_tbl.est_jump_m));
fprintf(f, '- max contradiction counter: `%d`\n', round(max(log_tbl.localize_contradiction_counter)));
fprintf(f, '- max no-progress counter: `%d`\n', round(max(log_tbl.localize_no_progress_counter)));
fprintf(f, '- max trace-loop counter: `%d`\n', round(max(log_tbl.localize_trace_loop_counter)));
fprintf(f, '- max commit watchdog counter: `%d`\n', round(max(log_tbl.commit_watchdog_counter)));
fprintf(f, '- max hard path distance counter: `%d`\n', round(max(log_tbl.hard_path_dist_counter)));
fprintf(f, '- reseed attempts / successes: `%d / %d`\n', round(max(log_tbl.reseed_attempt_count)), round(max(log_tbl.reseed_success_count)));
fprintf(f, '- max reseed crisis score: `%.3f`\n', max(log_tbl.reseed_last_score));
fprintf(f, '- max disambiguation goal switches: `%d`\n', round(max(log_tbl.disambiguation_goal_switches)));
fprintf(f, '- min path quality score: `%.3f`\n', min(log_tbl.path_quality_score));
fprintf(f, '- min trace bbox diag: `%.3f m`\n', min(log_tbl.localize_trace_bbox_diag_m));
fprintf(f, '- final estimated goal distance: `%.3f m`\n', log_tbl.goal_dist_est_m(end));
fprintf(f, '- final true goal distance: `%.3f m`\n\n', log_tbl.goal_dist_true_m(end));

fprintf(f, '## Stavy\n\n');
states = categories(categorical(log_tbl.nav_state));
for i = 1:numel(states)
    count_i = sum(strcmp(log_tbl.nav_state, states{i}));
    fprintf(f, '- `%s`: `%d` kroku\n', states{i}, count_i);
end

fprintf(f, '\n## Nejcastejsi reasons\n\n');
reasons = categorical(log_tbl.reason);
cats = categories(reasons);
counts = countcats(reasons);
[counts, order] = sort(counts, 'descend');
cats = cats(order);
top_n = min(5, numel(cats));
for i = 1:top_n
    fprintf(f, '- `%s`: `%d`\n', cats{i}, counts(i));
end

fprintf(f, '\n## Nejcastejsi transition reasons\n\n');
transitions = categorical(log_tbl.transition_reason);
cats = categories(transitions);
counts = countcats(transitions);
[counts, order] = sort(counts, 'descend');
cats = cats(order);
top_n = min(8, numel(cats));
for i = 1:top_n
    fprintf(f, '- `%s`: `%d`\n', cats{i}, counts(i));
end

fprintf(f, '\n## Nejcastejsi forced replans\n\n');
replans = categorical(log_tbl.force_replan_reason);
cats = categories(replans);
counts = countcats(replans);
[counts, order] = sort(counts, 'descend');
cats = cats(order);
top_n = min(8, numel(cats));
for i = 1:top_n
    if counts(i) > 0
        fprintf(f, '- `%s`: `%d`\n', cats{i}, counts(i));
    end
end

fprintf(f, '\n## Nejcastejsi forced relocalize\n\n');
relocalize = categorical(log_tbl.force_relocalize_reason);
cats = categories(relocalize);
counts = countcats(relocalize);
[counts, order] = sort(counts, 'descend');
cats = cats(order);
top_n = min(8, numel(cats));
for i = 1:top_n
    if counts(i) > 0
        fprintf(f, '- `%s`: `%d`\n', cats{i}, counts(i));
    end
end
end

function live_view = init_debug_live_view(map, agent_pose)
fig = figure('Name', 'Project tuning debug live view', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [80 80 1600 900]);
fig.ToolBar = 'figure';
ax = axes(fig, 'Position', [0.06 0.10 0.68 0.82]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
axis(ax, [map.limits(1)-1 map.limits(3)+1 map.limits(2)-1 map.limits(4)+1]);
title(ax, 'Project tuning debug live view', 'FontWeight', 'bold');

[cx, cy] = local_create_circle_dbg(map.goal(1), map.goal(2), map.goal_tolerance);
plot(ax, cx, cy, 'Color', 'green', 'LineWidth', 2, 'HandleVisibility', 'off');
[sx, sy] = local_create_arrow_dbg(agent_pose(1:2), agent_pose(3), 0.55);
plot(ax, sx, sy, 'Color', [0.10 0.10 0.10], 'LineWidth', 2.2);
[axp, ayp] = local_create_arrow_dbg(agent_pose(1:2), agent_pose(3), 0.5);
h_agent = plot(ax, axp, ayp, 'Color', 'blue', 'LineWidth', 2);
h_path = plot(ax, nan, nan, '-', 'Color', [0.10 0.80 0.20], 'LineWidth', 2.6);
h_est = plot(ax, nan, nan, 'r-', 'LineWidth', 1.8);
h_traj = plot(ax, nan, nan, '-', 'Color', [0.10 0.80 0.90], 'LineWidth', 2.0);
h_hyp = gobjects(3, 1);
h_hyp_text = gobjects(3, 1);
hyp_colors = [0.95 0.55 0.10; 0.85 0.20 0.65; 0.45 0.25 0.95];
for i = 1:3
    h_hyp(i) = plot(ax, nan, nan, 'o', ...
        'MarkerSize', 9 - i, ...
        'LineWidth', 1.8, ...
        'MarkerEdgeColor', hyp_colors(i, :), ...
        'MarkerFaceColor', 'none', ...
        'HandleVisibility', 'off');
    h_hyp_text(i) = text(ax, nan, nan, "", ...
        'Color', hyp_colors(i, :), ...
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'bottom', ...
        'Clipping', 'on');
end

for i = 1:size(map.walls, 1)
    line(ax, [map.walls(i,1), map.walls(i,3)], [map.walls(i,2), map.walls(i,4)], ...
        'Color', 'black', 'LineWidth', 5);
end
for i = 1:size(map.gnss_denied, 1)
    pgon = polyshape(map.gnss_denied(i, 1:2:end), map.gnss_denied(i, 2:2:end));
    plot(ax, pgon, 'FaceColor', 'magenta', 'FaceAlpha', 0.1, 'EdgeColor', 'magenta', 'LineStyle', '--');
end

panel = uipanel(fig, 'Title', 'Run Diagnostics', 'FontWeight', 'bold', ...
    'BackgroundColor', 'white', 'Position', [0.76 0.08 0.22 0.84]);
status_text = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05 0.03 0.90 0.94], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'white', ...
    'FontName', 'Consolas', 'FontSize', 12, 'String', '');

live_view = struct('figure', fig, 'axes', ax, 'agent_handle', h_agent, ...
    'path_handle', h_path, 'estimate_handle', h_est, 'traj_handle', h_traj, ...
    'hypothesis_handles', h_hyp, 'hypothesis_text_handles', h_hyp_text, ...
    'status_text', status_text);
end

function live_view = update_debug_live_view(live_view, private_vars, read_only_vars, public_vars)
if ~isgraphics(live_view.figure)
    return;
end
[axp, ayp] = local_create_arrow_dbg(private_vars.agent_pose(1:2), private_vars.agent_pose(3), 0.5);
set(live_view.agent_handle, 'XData', axp, 'YData', ayp);
path_to_draw = [];
if isfield(public_vars, 'display_path') && ~isempty(public_vars.display_path)
    path_to_draw = public_vars.display_path;
elseif isfield(public_vars, 'global_path') && ~isempty(public_vars.global_path)
    path_to_draw = public_vars.global_path;
elseif isfield(public_vars, 'path') && ~isempty(public_vars.path)
    path_to_draw = public_vars.path;
end
if ~isempty(path_to_draw)
    set(live_view.path_handle, 'XData', path_to_draw(:,1), 'YData', path_to_draw(:,2));
end
est = read_only_vars.est_position_history;
est = est(all(isfinite(est(:,1:2)), 2), :);
if ~isempty(est)
    set(live_view.estimate_handle, 'XData', est(:,1), 'YData', est(:,2));
end
if ~isempty(private_vars.agent_position_history)
    set(live_view.traj_handle, 'XData', private_vars.agent_position_history(:,1), ...
        'YData', private_vars.agent_position_history(:,2));
end

[v_cmd, w_cmd] = motion_vector_to_vw(public_vars.motion_vector, read_only_vars.agent_drive.interwheel_dist);
[front_min, left_min, right_min] = lidar_sector_minima_dbg(read_only_vars);
[path_dist, path_idx] = current_path_distance(public_vars);
q = get_field_or(public_vars, 'localization_quality', struct());
hyp_struct = get_nested_or(q, 'top_hypotheses', struct([]));
for i = 1:numel(live_view.hypothesis_handles)
    if i <= numel(hyp_struct) && isfield(hyp_struct(i), 'pose') && numel(hyp_struct(i).pose) >= 2 ...
            && all(isfinite(hyp_struct(i).pose(1:2)))
        xh = hyp_struct(i).pose(1);
        yh = hyp_struct(i).pose(2);
        set(live_view.hypothesis_handles(i), 'XData', xh, 'YData', yh, 'Visible', 'on');
        label = sprintf('H%d', i);
        if isfield(hyp_struct(i), 'mass') && isfinite(hyp_struct(i).mass)
            label = sprintf('H%d %.2f', i, hyp_struct(i).mass);
        end
        set(live_view.hypothesis_text_handles(i), 'Position', [xh + 0.10, yh + 0.08, 0], ...
            'String', label, 'Visible', 'on');
    else
        set(live_view.hypothesis_handles(i), 'XData', nan, 'YData', nan, 'Visible', 'off');
        set(live_view.hypothesis_text_handles(i), 'Position', [nan, nan, 0], 'String', "", 'Visible', 'off');
    end
end
reason = get_nested_or(q, 'reason', "");
dbg = get_field_or(public_vars, 'debug', struct());
transition_reason = get_nested_or(dbg, 'last_transition_reason', "");
track_event = get_nested_or(dbg, 'last_track_event', "");
force_replan_reason = get_nested_or(dbg, 'force_replan_reason', "");
force_relocalize_reason = get_nested_or(dbg, 'force_relocalize_reason', "");
motion_dbg = get_field_or(public_vars, 'motion_debug', struct());
speed_limiter = get_nested_or(motion_dbg, 'limiter', "none");
align_active = get_nested_or(motion_dbg, 'align_active', false);
align_target_heading = get_nested_or(motion_dbg, 'align_target_heading', nan);
align_heading_error = get_nested_or(motion_dbg, 'align_heading_error', nan);
ctrl_nearest_idx = get_nested_or(motion_dbg, 'nearest_idx', nan);
ctrl_look_idx = get_nested_or(motion_dbg, 'look_idx', nan);
ctrl_target_x = get_nested_or(motion_dbg, 'target_x', nan);
ctrl_target_y = get_nested_or(motion_dbg, 'target_y', nan);
ctrl_alpha = get_nested_or(motion_dbg, 'alpha', nan);
ctrl_curvature = get_nested_or(motion_dbg, 'curvature', nan);
ctrl_lookahead = get_nested_or(motion_dbg, 'lookahead_dist', nan);
pf_cluster = get_nested_or(q, 'pf_cluster_radius_m', nan);
pf_mass = get_nested_or(q, 'pf_dominant_mass', nan);
pf_ratio = get_nested_or(q, 'pf_top_ratio', nan);
pf_hypothesis_count = get_nested_or(q, 'pf_hypothesis_count', nan);
kf_var = get_nested_or(q, 'kf_var_xy_mean', nan);
disagree = get_nested_or(q, 'disagreement_xy_m', nan);
scan_mismatch_cost = get_nested_or(q, 'track_scan_mismatch_cost', nan);
scan_mismatch_counter = get_nested_or(q, 'track_scan_mismatch_counter', nan);
map_conflict_counter = get_nested_or(q, 'track_map_conflict_counter', nan);
loc_err = norm(public_vars.estimated_pose(1:2) - private_vars.agent_pose(1:2));
amb = get_field_or(public_vars, 'environment_ambiguity', struct());
amb_active = get_nested_or(amb, 'active', false);
amb_counter = get_nested_or(amb, 'counter', 0);
amb_scan = get_nested_or(amb, 'scan_change', nan);
clear_counter_effective = get_nested_or(amb, 'clear_counter', get_nested_or(public_vars.ambiguity, 'clear_counter', nan));
contradiction_counter = get_nested_or(public_vars.localize, 'contradiction_counter', nan);
no_progress_counter = get_nested_or(public_vars.localize, 'no_progress_counter', nan);
trace_loop_counter = get_nested_or(public_vars.localize, 'trace_loop_counter', nan);
trace_bbox_diag = get_nested_or(public_vars.localize, 'trace_bbox_diag_m', nan);
allow_global_reseed = get_nested_or(public_vars.localize, 'allow_global_reseed', false);
localization_commit_state = string(get_nested_or(get_field_or(public_vars, 'localization_commit', struct()), 'state', "search"));
commit_watchdog_counter = get_nested_or(q, 'commit_watchdog_counter', nan);
hard_path_dist_counter = get_nested_or(q, 'hard_path_dist_counter', nan);
reseed_attempt_count = get_nested_or(q, 'reseed_attempt_count', nan);
reseed_success_count = get_nested_or(q, 'reseed_success_count', nan);
reseed_last_score = get_nested_or(q, 'reseed_last_score', nan);
post_reseed_hold_counter = get_nested_or(q, 'post_reseed_hold_counter', nan);
disambiguation_goal_switches = get_nested_or(q, 'disambiguation_goal_switches', nan);
path_quality_score = get_nested_or(get_field_or(public_vars, 'path_quality', struct()), 'score', nan);
scan_match_ema = get_nested_or(q, 'scan_match_ema', nan);
path_distance_ema = get_nested_or(q, 'path_distance_ema', nan);
h1_mass = get_nested_or(q, 'h1_mass', nan);
h1_score = get_nested_or(q, 'h1_scan_score', nan);
h1_stability = get_nested_or(q, 'h1_stability', nan);
h2_mass = get_nested_or(q, 'h2_mass', nan);
h2_score = get_nested_or(q, 'h2_scan_score', nan);
h2_stability = get_nested_or(q, 'h2_stability', nan);
h3_mass = get_nested_or(q, 'h3_mass', nan);
h3_score = get_nested_or(q, 'h3_scan_score', nan);
h3_stability = get_nested_or(q, 'h3_stability', nan);
est_jump_m = nan;
if size(read_only_vars.est_position_history, 1) >= 2
    prev_est = read_only_vars.est_position_history(end - 1, 1:2);
    curr_est = read_only_vars.est_position_history(end, 1:2);
    if all(isfinite(prev_est)) && all(isfinite(curr_est))
        est_jump_m = norm(curr_est - prev_est);
    end
end

live_view.status_text.String = sprintf([ ...
    '%-14s %5d\n%-14s %s\n%-14s %s\n%-14s %s\n\n' ...
    '%-14s %7.3f m/s\n%-14s %7.3f rad/s\n%-14s %s\n\n' ...
    '%-14s %4.0f\n%-14s %s\n%-14s %7.3f m\n%-14s %7.3f\n%-14s %7.3f\n' ...
    '%-14s %7.3f\n%-14s %7.3f\n%-14s %7.3f m\n%-14s %4d\n' ...
    '%-14s %4.2f/%4.2f/%4.2f\n%-14s %4.2f/%4.2f/%4.2f\n%-14s %4.2f/%4.2f/%4.2f\n\n' ...
    '%-14s %s\n%-14s %s\n%-14s %s\n%-14s %s\n\n' ...
    '%-14s %7.3f / %7.3f / %7.3f\n%-14s %7.3f / %7.3f\n%-14s %4d / %4d\n' ...
    '%-14s %7.3f / %7.3f\n%-14s %7.3f / %7.3f\n%-14s %7.3f\n\n' ...
    '%-14s %7.3f / %3d\n%-14s %3d\n%-14s %3d / %3.0f\n%-14s %7.3f m\n' ...
    '%-14s %3.0f / %3.0f\n%-14s %3.0f / %7.3f\n%-14s %d / %7.3f m\n' ...
    '%-14s %3.0f / %3.0f\n%-14s %7.3f / %7.3f\n%-14s %7.3f / %7.3f'], ...
    'step:', read_only_vars.counter, ...
    'state:', string(get_field_or(public_vars, 'nav_state', "unknown")), ...
    'reason:', string(reason), ...
    'commit:', string(localization_commit_state), ...
    'v:', v_cmd, ...
    'w:', w_cmd, ...
    'limiter:', string(speed_limiter), ...
    'hypotheses:', pf_hypothesis_count, ...
    'transition:', string(transition_reason), ...
    'loc err:', loc_err, ...
    'pf cluster:', pf_cluster, ...
    'pf mass:', pf_mass, ...
    'pf ratio:', pf_ratio, ...
    'kf var:', kf_var, ...
    'disagree:', disagree, ...
    'path dist:', path_dist, ...
    'path idx:', path_idx, ...
    'H1 m/s/st:', h1_mass, h1_score, h1_stability, ...
    'H2 m/s/st:', h2_mass, h2_score, h2_stability, ...
    'H3 m/s/st:', h3_mass, h3_score, h3_stability, ...
    'track event:', string(track_event), ...
    'replan:', string(force_replan_reason), ...
    'relocalize:', string(force_relocalize_reason), ...
    'ambiguity:', string(logical(amb_active)), ...
    'front/l/r:', front_min, left_min, right_min, ...
    'align th/e:', align_target_heading, align_heading_error, ...
    'near/look:', ctrl_nearest_idx, ctrl_look_idx, ...
    'target xy:', ctrl_target_x, ctrl_target_y, ...
    'alpha/curv:', ctrl_alpha, ctrl_curvature, ...
    'lookahead:', ctrl_lookahead, ...
    'scan mm:', scan_mismatch_cost, scan_mismatch_counter, ...
    'map/clear:', map_conflict_counter, clear_counter_effective, ...
    'amb ctr/chg:', amb_counter, amb_scan, ...
    'contr/noProg:', contradiction_counter, no_progress_counter, ...
    'trace/bbox:', trace_loop_counter, trace_bbox_diag, ...
    'reseed/jump:', allow_global_reseed, est_jump_m, ...
    'reseeds ok/all:', reseed_success_count, reseed_attempt_count, ...
    'watch/path ctr:', commit_watchdog_counter, hard_path_dist_counter, ...
    'EMA scan/path:', scan_match_ema, path_distance_ema, ...
    'path q/switch:', path_quality_score, disambiguation_goal_switches);
drawnow limitrate;
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

function [front, left, right] = lidar_sector_minima_dbg(read_only_vars)
front = inf; left = inf; right = inf;
if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances)
    return;
end
d = read_only_vars.lidar_distances(:);
a = read_only_vars.lidar_config(:);
valid = isfinite(d) & isfinite(a);
d = d(valid);
a = wrap_to_pi_dbg(a(valid));
front_mask = abs(a) <= 35 * pi / 180;
left_mask = a > 20 * pi / 180 & a < 160 * pi / 180;
right_mask = a < -20 * pi / 180 & a > -160 * pi / 180;
if any(front_mask), front = min(d(front_mask)); end
if any(left_mask), left = min(d(left_mask)); end
if any(right_mask), right = min(d(right_mask)); end
end

function [dist, idx] = current_path_distance(public_vars)
dist = inf;
idx = 1;
if ~isfield(public_vars, 'path') || isempty(public_vars.path) ...
        || ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose)
    return;
end
d = vecnorm(public_vars.path(:, 1:2) - public_vars.estimated_pose(1:2), 2, 2);
[dist, idx] = min(d);
end

function value = get_field_or(s, field_name, fallback)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = fallback;
end
end

function value = get_nested_or(s, field_name, fallback)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = fallback;
end
end

function a = wrap_to_pi_dbg(a)
a = mod(a + pi, 2 * pi) - pi;
end

function [x, y] = local_create_circle_dbg(cx, cy, r)
t = linspace(0, 2 * pi, 100);
x = cx + r * cos(t);
y = cy + r * sin(t);
end

function [x, y] = local_create_arrow_dbg(p, theta, len)
tip = p(:)' + len * [cos(theta), sin(theta)];
left = p(:)' + 0.35 * len * [cos(theta + 2.5), sin(theta + 2.5)];
right = p(:)' + 0.35 * len * [cos(theta - 2.5), sin(theta - 2.5)];
x = [p(1), tip(1), left(1), tip(1), right(1)];
y = [p(2), tip(2), left(2), tip(2), right(2)];
end

function rel = rel_path(path_abs, root_abs)
rel = string(path_abs);
prefix = [root_abs filesep];
if startsWith(path_abs, prefix, 'IgnoreCase', true)
    rel = string(extractAfter(path_abs, strlength(prefix)));
end
rel = char(rel);
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
