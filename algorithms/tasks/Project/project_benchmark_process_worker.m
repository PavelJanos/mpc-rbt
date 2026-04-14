function project_benchmark_process_worker(task_file, result_file)
worker_result = struct('ok', false, 'status', "worker_error", 'message', "", 'session_dir', "", 'task_result', []);
final_result_file = result_file;
try
    loaded = load(task_file, 'spec');
    spec = loaded.spec;
    cd(spec.project_dir);
    worker_started = datetime('now');

    setenv('PROJECT_BENCHMARK_MODE', 'test');
    setenv('PROJECT_TEST_MAP', char(string(spec.map_name)));
    setenv('PROJECT_NUM_REPEATS', '1');
    setenv('PROJECT_TIMEOUT_STEPS', sprintf('%d', spec.timeout_steps));
    setenv('PROJECT_VIEW_MODE', '0');
    setenv('PROJECT_STOP_MODE', 'all');
    setenv('PROJECT_CONCURRENT_RUNS', '1');
    setenv('PROJECT_CASE_MODE', 'reuse');
    setenv('PROJECT_CASES_FILE', char(string(spec.cases_file)));
    setenv('PROJECT_SESSION_LABEL', char(string(spec.session_label)));
    setenv('PROJECT_BENCHMARK_WARM', '1');

    project_test_all_maps;

    actual_session_dir = resolve_created_session_dir(spec.project_dir, "benchmark", string(spec.session_label), worker_started);
    detail_file = fullfile(actual_session_dir, 'project_test_all_maps_detail.csv');
    if ~isfile(detail_file)
        error('Benchmark worker detail table not found: %s', detail_file);
    end
    detail_tbl = readtable(detail_file);
    if height(detail_tbl) < 1
        error('Benchmark worker detail table is empty: %s', detail_file);
    end
    row = detail_tbl(1, :);
    task_result = struct( ...
        'map_name', string(row.map_name(1)), ...
        'repeat_id', row.repeat_id(1), ...
        'start_pose', [row.start_x(1), row.start_y(1), row.start_theta(1)], ...
        'goal_xy', [row.goal_x(1), row.goal_y(1)], ...
        'max_vel', row.max_vel(1), ...
        'cases_mode', string(row.cases_mode(1)), ...
        'status', string(row.status(1)), ...
        'success', logical(row.success(1)), ...
        'steps', row.steps(1), ...
        'goal_error_m', row.goal_error_m(1), ...
        'travelled_m', row.travelled_m(1), ...
        'runtime_s', row.runtime_s(1), ...
        'final_pose', [row.final_x(1), row.final_y(1), row.final_theta(1)], ...
        'path', zeros(0, 2), ...
        'pose_history', zeros(0, 3), ...
        'est_history', zeros(0, 3));
    replay_file = fullfile(actual_session_dir, 'HistoryMapsReview', sprintf('benchmark_%s_run_%02d_replay.mat', char(spec.map_name), spec.repeat_id));
    if isfile(replay_file)
        replay_loaded = load(replay_file, 'task_result_artifact');
        if isfield(replay_loaded, 'task_result_artifact')
            replay_result = replay_loaded.task_result_artifact;
            if isfield(replay_result, 'path') && ~isempty(replay_result.path)
                task_result.path = replay_result.path;
            end
            if isfield(replay_result, 'pose_history') && ~isempty(replay_result.pose_history)
                task_result.pose_history = replay_result.pose_history;
            end
            if isfield(replay_result, 'est_history') && ~isempty(replay_result.est_history)
                task_result.est_history = replay_result.est_history;
            end
        end
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
