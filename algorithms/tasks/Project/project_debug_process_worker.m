function project_debug_process_worker(task_file, result_file)
worker_result = struct('ok', false, 'status', "worker_error", 'message', "", 'session_dir', "", 'task_result', []);
final_result_file = result_file;
try
    loaded = load(task_file, 'spec');
    spec = loaded.spec;
    cd(spec.project_dir);
    worker_started = datetime('now');

    setenv('PROJECT_DEBUG_MAP_LIST', char(string(spec.map_name)));
    setenv('PROJECT_DEBUG_REPEAT_LIST', sprintf('%d', spec.repeat_id));
    setenv('PROJECT_DEBUG_VIEW', '0');
    setenv('PROJECT_DEBUG_TIMEOUT', sprintf('%d', spec.timeout_steps));
    setenv('PROJECT_CONTROLLER_MODE', char(string(spec.controller_mode)));
    setenv('PROJECT_SMOOTHING_MODE', char(string(spec.path_smoothing_mode)));
    setenv('PROJECT_LOCALIZATION_MODE', char(string(spec.localization_mode)));
    setenv('PROJECT_DEBUG_SAVE_OUTPUTS', ternary_env(spec.save_outputs));
    setenv('PROJECT_DEBUG_CONCURRENT_RUNS', '1');
    setenv('PROJECT_SESSION_LABEL', char(string(spec.session_label)));
    setenv('PROJECT_DEBUG_WARM', '1');
    if isfield(spec, 'cases_file') && ~isempty(spec.cases_file)
        setenv('PROJECT_CASES_FILE', char(string(spec.cases_file)));
    end

    project_tuning_debug;

    actual_session_dir = resolve_created_session_dir(spec.project_dir, "debug", string(spec.session_label), worker_started);
    summary_file = fullfile(actual_session_dir, 'project_tuning_debug_runs.csv');
    if ~isfile(summary_file)
        error('Debug worker summary not found: %s', summary_file);
    end
    summary_tbl = readtable(summary_file, 'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');
    if height(summary_tbl) < 1
        error('Debug worker summary is empty: %s', summary_file);
    end
    run_dir = char(summary_tbl{1, 'session_subdir'});
    log_file = fullfile(run_dir, sprintf('project_tuning_debug_%s_run_%02d_log.csv', char(spec.map_name), spec.repeat_id));
    paths_file = fullfile(run_dir, sprintf('project_tuning_debug_%s_run_%02d_paths.csv', char(spec.map_name), spec.repeat_id));
    log_tbl = table();
    path_tbl = table();
    if isfile(log_file)
        log_tbl = readtable(log_file);
    end
    if isfile(paths_file)
        path_tbl = readtable(paths_file);
    end

    task_result = struct( ...
        'map_name', string(summary_tbl{1, 'map_name'}), ...
        'repeat_id', summary_tbl{1, 'repeat_id'}, ...
        'start_pose', [nan nan nan], ...
        'goal_xy', [nan nan], ...
        'status', string(summary_tbl{1, 'status'}), ...
        'steps', summary_tbl{1, 'steps'}, ...
        'goal_error_m', summary_tbl{1, 'goal_error_m'}, ...
        'travelled_m', summary_tbl{1, 'travelled_m'}, ...
        'runtime_s', summary_tbl{1, 'runtime_s'}, ...
        'run_dir', string(run_dir), ...
        'log_tbl', log_tbl, ...
        'path_tbl', path_tbl);
    if isfield(spec, 'start_pose')
        task_result.start_pose = spec.start_pose;
    end
    if isfield(spec, 'goal_xy')
        task_result.goal_xy = spec.goal_xy;
    end

    worker_result.ok = true;
    worker_result.status = string(task_result.status);
    worker_result.session_dir = string(actual_session_dir);
    worker_result.task_result = task_result;
catch ME
    worker_result.message = string(getReport(ME, 'extended', 'hyperlinks', 'off'));
    if isfield(worker_result, 'session_dir') && strlength(string(worker_result.session_dir)) == 0 && exist('spec', 'var') && isfield(spec, 'session_dir')
        worker_result.session_dir = string(spec.session_dir);
    end
end
temp_result_file = final_result_file + ".tmp";
save(char(temp_result_file), 'worker_result', '-v7');
movefile(char(temp_result_file), char(final_result_file), 'f');
end

function session_dir = resolve_created_session_dir(project_dir, prefix, session_label, worker_started)
sessions_root = fullfile(project_dir, 'RunSessions');
pattern = sprintf('%s_%s*', char(prefix), char(session_label));
dirs = dir(fullfile(sessions_root, pattern));
dirs = dirs([dirs.isdir]);
if isempty(dirs)
    session_dir = fullfile(sessions_root, sprintf('%s_%s', char(prefix), char(session_label)));
    return;
end
best_idx = 1;
best_score = inf;
for i = 1:numel(dirs)
    created = datetime(dirs(i).datenum, 'ConvertFrom', 'datenum');
    score = abs(seconds(created - worker_started));
    if score < best_score
        best_score = score;
        best_idx = i;
    end
end
session_dir = fullfile(dirs(best_idx).folder, dirs(best_idx).name);
end

function out = ternary_env(value)
if value
    out = '1';
else
    out = '0';
end
end
