%% Project tuning triplet benchmark
clear; clc; close all;

project_dir = fileparts(mfilename('fullpath'));
triplet_maps = {'indoor_1', 'indoor_2', 'outdoor_1'};
repeat_id = 1;
view_mode = false;
controller_mode = getenv_default('PROJECT_CONTROLLER_MODE', 'pure_pursuit');
path_smoothing_mode = getenv_default('PROJECT_SMOOTHING_MODE', 'chaikin');
localization_mode = getenv_default('PROJECT_LOCALIZATION_MODE', 'fusion');

rows = cell(numel(triplet_maps), 11);

for i = 1:numel(triplet_maps)
    map_label = triplet_maps{i};
    ensure_test_case_file(project_dir, map_label, repeat_id);
    run_debug_subprocess(project_dir, map_label, repeat_id, view_mode, controller_mode, path_smoothing_mode, localization_mode);

    log_path = fullfile(project_dir, sprintf('project_tuning_debug_%s_run_%02d_log.csv', map_label, repeat_id));
    report_path = fullfile(project_dir, sprintf('project_tuning_debug_%s_run_%02d_report.md', map_label, repeat_id));
    log_tbl = readtable(log_path);
    report_lines = string(splitlines(fileread(report_path)));

    status = extract_md_value(report_lines, '- vysledek:');
    goal_error = parse_md_numeric(report_lines, '- final goal error:');
    travelled = parse_md_numeric(report_lines, '- travelled:');
    runtime_s = parse_md_numeric(report_lines, '- runtime:');
    dominant_state = dominant_string(log_tbl, 'nav_state');
    dominant_limiter = dominant_string(log_tbl, 'speed_limiter');
    dominant_replan = dominant_string_filtered(log_tbl, 'force_replan_reason', "none");
    dominant_relocalize = dominant_string_filtered(log_tbl, 'force_relocalize_reason', "none");
    mean_track_speed = mean_for_state(log_tbl, 'track', 'v_cmd');
    mean_track_path_dist = mean_for_state(log_tbl, 'track', 'path_distance_m');
    mean_track_front = mean_for_state(log_tbl, 'track', 'front_min');

    rows(i, :) = {map_label, status, goal_error, travelled, runtime_s, dominant_state, dominant_limiter, ...
        dominant_replan, dominant_relocalize, mean_track_speed, mean_track_path_dist};

    fprintf('Triplet run complete: %s | %s | goal err %.3f m\n', map_label, status, goal_error);
end

triplet_tbl = cell2table(rows, 'VariableNames', { ...
    'map_name', 'status', 'goal_error_m', 'travelled_m', 'runtime_s', ...
    'dominant_state', 'dominant_limiter', 'dominant_replan', 'dominant_relocalize', ...
    'mean_track_speed_mps', 'mean_track_path_distance_m'});

csv_path = fullfile(project_dir, 'project_tuning_triplet_summary.csv');
writetable(triplet_tbl, csv_path);

report_path = fullfile(project_dir, 'project_tuning_triplet_report.md');
write_triplet_report(report_path, triplet_tbl, triplet_maps, repeat_id, controller_mode, path_smoothing_mode, localization_mode);

fprintf('Saved: %s\n', rel_path_local(csv_path, project_dir));
fprintf('Saved: %s\n', rel_path_local(report_path, project_dir));

function s = logical_to_env(tf)
if tf
    s = 'true';
else
    s = 'false';
end
end

function value = extract_md_value(lines, prefix)
line = lines(startsWith(strtrim(lines), prefix));
if isempty(line)
    value = "";
    return;
end
value = strtrim(extractAfter(line(1), prefix));
value = erase(value, '`');
end

function value = parse_md_numeric(lines, prefix)
text_value = extract_md_value(lines, prefix);
token = regexp(text_value, '[-+]?[0-9]*\.?[0-9]+', 'match', 'once');
if isempty(token)
    value = nan;
else
    value = str2double(token);
end
end

function value = dominant_string(tbl, var_name)
if ~ismember(var_name, tbl.Properties.VariableNames)
    value = "";
    return;
end
vals = string(tbl.(var_name));
vals(vals == "") = missing;
vals = vals(~ismissing(vals));
if isempty(vals)
    value = "";
    return;
end
u = unique(vals);
counts = zeros(size(u));
for k = 1:numel(u)
    counts(k) = sum(vals == u(k));
end
[~, idx] = max(counts);
value = u(idx);
end

function value = dominant_string_filtered(tbl, var_name, ignore_value)
if ~ismember(var_name, tbl.Properties.VariableNames)
    value = "";
    return;
end
vals = string(tbl.(var_name));
vals(vals == "" | vals == ignore_value) = missing;
vals = vals(~ismissing(vals));
if isempty(vals)
    value = "none";
    return;
end
u = unique(vals);
counts = zeros(size(u));
for k = 1:numel(u)
    counts(k) = sum(vals == u(k));
end
[~, idx] = max(counts);
value = u(idx);
end

function value = mean_for_state(tbl, state_name, var_name)
value = nan;
if ~ismember('nav_state', tbl.Properties.VariableNames) || ~ismember(var_name, tbl.Properties.VariableNames)
    return;
end
mask = strcmp(string(tbl.nav_state), state_name) & isfinite(tbl.(var_name));
if any(mask)
    value = mean(tbl.(var_name)(mask));
end
end

function write_triplet_report(report_path, triplet_tbl, triplet_maps, repeat_id, controller_mode, path_smoothing_mode, localization_mode)
fid = fopen(report_path, 'w');
assert(fid ~= -1, 'Failed to open report file for writing.');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Project tuning triplet\n\n');
fprintf(fid, '- mapy: `%s`\n', strjoin(triplet_maps, ', '));
fprintf(fid, '- repeat id: `%d`\n', repeat_id);
fprintf(fid, '- controller: `%s`\n', controller_mode);
fprintf(fid, '- smoothing: `%s`\n', path_smoothing_mode);
fprintf(fid, '- localization: `%s`\n\n', localization_mode);

fprintf(fid, '## Souhrn\n\n');
for i = 1:height(triplet_tbl)
    fprintf(fid, '- `%s`: status `%s`, goal error `%.3f m`, dominant state `%s`, dominant limiter `%s`, mean track speed `%.3f m/s`\n', ...
        triplet_tbl.map_name{i}, triplet_tbl.status{i}, triplet_tbl.goal_error_m(i), ...
        triplet_tbl.dominant_state{i}, triplet_tbl.dominant_limiter{i}, triplet_tbl.mean_track_speed_mps(i));
end

fprintf(fid, '\n## Spolecne slabe misto\n\n');
common_limiter = dominant_string(triplet_tbl, 'dominant_limiter');
common_state = dominant_string(triplet_tbl, 'dominant_state');
statuses = string(triplet_tbl.status);
timeout_count = sum(statuses == "timeout");
wall_count = sum(statuses == "wall");
fprintf(fid, '- nejcastejsi dominantni limiter: `%s`\n', common_limiter);
fprintf(fid, '- nejcastejsi dominantni stav: `%s`\n', common_state);
fprintf(fid, '- timeout behu: `%d/%d`\n', timeout_count, height(triplet_tbl));
fprintf(fid, '- wall behu: `%d/%d`\n', wall_count, height(triplet_tbl));
fprintf(fid, '- prumerna track rychlost napric mapami: `%.3f m/s`\n', mean(triplet_tbl.mean_track_speed_mps, 'omitnan'));
fprintf(fid, '- prumerna track path distance napric mapami: `%.3f m`\n', mean(triplet_tbl.mean_track_path_distance_m, 'omitnan'));
end

function ensure_test_case_file(project_dir, map_label, repeat_id)
test_case_file = fullfile(project_dir, sprintf('project_test_all_maps_cases_test_%s.csv', map_label));
if exist(test_case_file, 'file')
    return;
end

master_file = fullfile(project_dir, 'project_test_all_maps_cases.csv');
if ~exist(master_file, 'file')
    error('Missing master cases file: %s', master_file);
end

master_tbl = readtable(master_file);
mask = strcmp(master_tbl.map_name, map_label) & master_tbl.repeat_id == repeat_id;
if ~any(mask)
    error('No master case found for map %s repeat %d.', map_label, repeat_id);
end
test_tbl = master_tbl(mask, :);
writetable(test_tbl, test_case_file);
end

function run_debug_subprocess(project_dir, map_label, repeat_id, view_mode, controller_mode, path_smoothing_mode, localization_mode)
setenv('PROJECT_DEBUG_MAP', map_label);
setenv('PROJECT_DEBUG_REPEAT', num2str(repeat_id));
setenv('PROJECT_DEBUG_VIEW', logical_to_env(view_mode));
setenv('PROJECT_CONTROLLER_MODE', controller_mode);
setenv('PROJECT_SMOOTHING_MODE', path_smoothing_mode);
setenv('PROJECT_LOCALIZATION_MODE', localization_mode);
timeout_override = getenv('PROJECT_DEBUG_TIMEOUT');
if ~isempty(timeout_override)
    setenv('PROJECT_DEBUG_TIMEOUT', timeout_override);
end

project_root = project_dir;
while ~exist(fullfile(project_root, 'main.m'), 'file')
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        error('Could not locate project root.');
    end
    project_root = parent;
end

matlab_exe = 'C:\Program Files\MATLAB\R2023b\bin\matlab.exe';
batch_cmd = sprintf("project_root=pwd; addpath(genpath(fullfile(project_root,'algorithms'))); addpath(fullfile(project_root,'utils')); run(fullfile(project_root,'algorithms','tasks','Project','project_tuning_debug.m'));");
old_dir = pwd;
cleanup = onCleanup(@() cd(old_dir));
cd(project_root);
status = system(sprintf('"%s" -batch "%s"', matlab_exe, batch_cmd));
if status ~= 0
    error('Debug subprocess failed for map %s.', map_label);
end
end

function value = getenv_default(name, fallback)
value = getenv(name);
if isempty(value)
    value = fallback;
end
end

function rel = rel_path_local(path_value, root_dir)
rel = strrep(path_value, [root_dir filesep], '');
end
