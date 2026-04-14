function results = project_tuning_ga(cfg)
%PROJECT_TUNING_GA Headless genetic tuning over selected debug cases.
%   RESULTS = PROJECT_TUNING_GA(CFG) runs one or more independent genetic
%   algorithm searches. Each candidate is evaluated by project_tuning_debug
%   without live view and, by default, without per-run figures/CSVs.
%
%   Important CFG fields:
%   - mutable_params : struct array with fields:
%       name  - parameter path, e.g. 'public_vars.localize.pf_cluster_good'
%               or shortened 'localize.pf_cluster_good' (defaults to
%               public_vars.*)
%       lower - lower bound
%       upper - upper bound
%       type  - 'real' or 'integer'
%       initial (optional) - seed value for the first chromosome
%   - map_list, repeat_list, timeout_steps, num_runs, population_size,
%     generations, elite_count, mutation_rate, crossover_rate,
%     tournament_size, case_concurrency, session_label, random_seed
%
%   Example:
%   cfg = struct();
%   cfg.map_list = "indoor_2";
%   cfg.repeat_list = [1 2 3];
%   cfg.num_runs = 2;
%   cfg.population_size = 12;
%   cfg.generations = 8;
%   cfg.mutable_params = [ ...
%       struct('name',"localize.pf_cluster_good",'lower',0.20,'upper',0.60,'type',"real"), ...
%       struct('name',"localize.pf_dominant_mass_good",'lower',0.55,'upper',0.95,'type',"real"), ...
%       struct('name',"path_clearance_m",'lower',0.15,'upper',0.40,'type',"real")];
%   results = project_tuning_ga(cfg);

project_dir = fileparts(mfilename('fullpath'));
if ~strcmpi(pwd, project_dir)
    cd(project_dir);
end

cfg = normalize_ga_cfg(cfg, project_dir);
param_specs = normalize_param_specs(cfg.mutable_params);
project_root = locate_project_root(project_dir);
sessions_root = fullfile(project_dir, 'RunSessions');
session_dir = create_tuning_session_dir(sessions_root, 'ga', cfg.session_label);

fprintf('GA tuning session: %s\n', session_dir);
fprintf('Cases file: %s\n', cfg.cases_file);
fprintf('Maps: %s\n', strjoin(cellstr(cfg.map_list), ', '));
fprintf('Repeats: %s\n', sprintf('%d ', cfg.repeat_list));

all_eval_rows = struct([]);
generation_rows = struct([]);
run_rows = struct([]);
best_overall = struct('score', inf);
cache = containers.Map('KeyType', 'char', 'ValueType', 'any');

for run_idx = 1:cfg.num_runs
    rng(cfg.random_seed + run_idx - 1, 'twister');
    population = initialize_population(cfg.population_size, param_specs);
    run_best = struct('score', inf);

    for generation_idx = 1:cfg.generations
        evals = repmat(empty_eval_result(), cfg.population_size, 1);
        for individual_idx = 1:cfg.population_size
            values = population(individual_idx, :);
            evals(individual_idx) = evaluate_candidate(values, param_specs, cfg, cache, run_idx, generation_idx, individual_idx, project_dir);
        end

        [evals, population] = sort_population(evals, population);
        if evals(1).score < run_best.score
            run_best = evals(1);
        end
        if evals(1).score < best_overall.score
            best_overall = evals(1);
        end

        generation_rows = [generation_rows; build_generation_row(run_idx, generation_idx, evals)]; %#ok<AGROW>
        all_eval_rows = [all_eval_rows; build_eval_rows(run_idx, generation_idx, evals, param_specs)]; %#ok<AGROW>

        fprintf('Run %d/%d | Gen %d/%d | best=%.3f | success=%.2f | goalErr=%.3f | runtime=%.3f\n', ...
            run_idx, cfg.num_runs, generation_idx, cfg.generations, ...
            evals(1).score, evals(1).success_rate, evals(1).mean_goal_error_m, evals(1).mean_runtime_s);

        if generation_idx < cfg.generations
            population = breed_next_population(population, evals, param_specs, cfg);
        end
    end

    run_rows = [run_rows; build_run_row(run_idx, run_best, param_specs)]; %#ok<AGROW>
end

if isempty(best_overall) || ~isfinite(best_overall.score)
    error('GA tuning did not produce a valid candidate.');
end

eval_tbl = struct2table(all_eval_rows);
generation_tbl = struct2table(generation_rows);
run_tbl = struct2table(run_rows);
best_case_tbl = best_overall.summary_tbl;
best_case_tbl.score_component = compute_case_score_components(best_case_tbl, cfg);

writetable(eval_tbl, fullfile(session_dir, 'project_tuning_ga_evaluations.csv'));
writetable(generation_tbl, fullfile(session_dir, 'project_tuning_ga_generations.csv'));
writetable(run_tbl, fullfile(session_dir, 'project_tuning_ga_runs.csv'));
writetable(best_case_tbl, fullfile(session_dir, 'project_tuning_ga_best_cases.csv'));
save(fullfile(session_dir, 'project_tuning_ga_results.mat'), 'cfg', 'param_specs', 'best_overall', 'run_tbl', 'generation_tbl');
write_ga_report(fullfile(session_dir, 'project_tuning_ga_report.md'), cfg, param_specs, run_tbl, generation_tbl, best_overall);

results = struct( ...
    'session_dir', string(session_dir), ...
    'best_score', best_overall.score, ...
    'best_params', best_overall.param_struct, ...
    'best_values', best_overall.values, ...
    'best_summary', best_overall.summary_tbl, ...
    'run_table', run_tbl, ...
    'generation_table', generation_tbl, ...
    'evaluation_table', eval_tbl);

fprintf('Saved: %s\n', fullfile(session_dir, 'project_tuning_ga_report.md'));
fprintf('Best score: %.3f | success rate: %.2f | mean goal err: %.3f m\n', ...
    best_overall.score, best_overall.success_rate, best_overall.mean_goal_error_m);
end

function cfg = normalize_ga_cfg(cfg, project_dir)
if nargin < 1 || isempty(cfg)
    cfg = struct();
end

cfg = set_default(cfg, 'map_list', "indoor_2");
cfg = set_default(cfg, 'repeat_list', 1);
cfg = set_default(cfg, 'timeout_steps', 2050);
cfg = set_default(cfg, 'num_runs', 2);
cfg = set_default(cfg, 'population_size', 12);
cfg = set_default(cfg, 'generations', 8);
cfg = set_default(cfg, 'elite_count', 2);
cfg = set_default(cfg, 'mutation_rate', 0.25);
cfg = set_default(cfg, 'mutation_sigma_fraction', 0.18);
cfg = set_default(cfg, 'crossover_rate', 0.85);
cfg = set_default(cfg, 'tournament_size', 3);
cfg = set_default(cfg, 'case_concurrency', 1);
cfg = set_default(cfg, 'controller_mode', 'pure_pursuit');
cfg = set_default(cfg, 'path_smoothing_mode', 'chaikin');
cfg = set_default(cfg, 'localization_mode', 'fusion');
cfg = set_default(cfg, 'save_outputs', false);
cfg = set_default(cfg, 'random_seed', 1337);
cfg = set_default(cfg, 'session_label', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
cfg = set_default(cfg, 'mutable_params', default_param_specs());

cfg.map_list = normalize_text_list(cfg.map_list);
cfg.repeat_list = unique(round(cfg.repeat_list(:)'), 'stable');
cfg.population_size = max(4, round(cfg.population_size));
cfg.generations = max(1, round(cfg.generations));
cfg.num_runs = max(1, round(cfg.num_runs));
cfg.elite_count = min(max(1, round(cfg.elite_count)), cfg.population_size);
cfg.tournament_size = min(max(2, round(cfg.tournament_size)), cfg.population_size);
cfg.case_concurrency = max(1, round(cfg.case_concurrency));
cfg.timeout_steps = max(50, round(cfg.timeout_steps));
cfg.mutation_rate = min(max(cfg.mutation_rate, 0), 1);
cfg.crossover_rate = min(max(cfg.crossover_rate, 0), 1);
cfg.mutation_sigma_fraction = max(cfg.mutation_sigma_fraction, 1e-3);

if isfield(cfg, 'cases_file') && ~isempty(cfg.cases_file)
    cfg.cases_file = char(string(cfg.cases_file));
else
    if numel(cfg.map_list) == 1
        cfg.cases_file = fullfile(project_dir, sprintf('project_test_all_maps_cases_test_%s.csv', char(cfg.map_list)));
    else
        cfg.cases_file = fullfile(project_dir, 'project_test_all_maps_cases_full.csv');
    end
end

if ~isfile(cfg.cases_file)
    error('Cases file not found: %s. Run project_test_all_maps.m first.', cfg.cases_file);
end
end

function specs = default_param_specs()
specs = [ ...
    struct('name', "localize.pf_cluster_good", 'lower', 0.22, 'upper', 0.60, 'type', "real"), ...
    struct('name', "localize.pf_dominant_mass_good", 'lower', 0.55, 'upper', 0.95, 'type', "real"), ...
    struct('name', "localize.fast_exit_pf_cluster", 'lower', 0.25, 'upper', 0.60, 'type', "real"), ...
    struct('name', "path_clearance_m", 'lower', 0.15, 'upper', 0.40, 'type', "real"), ...
    struct('name', "path_resample_ds_m", 'lower', 0.06, 'upper', 0.22, 'type', "real"), ...
    struct('name', "smooth_chaikin_iters", 'lower', 1, 'upper', 6, 'type', "integer")];
end

function specs = normalize_param_specs(specs)
if isempty(specs)
    error('mutable_params must not be empty.');
end
if ~isstruct(specs)
    error('mutable_params must be a struct array.');
end

specs = specs(:);
for i = 1:numel(specs)
    if ~isfield(specs(i), 'name') || ~isfield(specs(i), 'lower') || ~isfield(specs(i), 'upper')
        error('Each mutable param needs fields name/lower/upper.');
    end
    if ~isfield(specs(i), 'type') || isempty(specs(i).type)
        specs(i).type = "real";
    end
    specs(i).name = normalize_param_name(specs(i).name);
    specs(i).lower = double(specs(i).lower);
    specs(i).upper = double(specs(i).upper);
    specs(i).type = lower(string(specs(i).type));
    if specs(i).upper < specs(i).lower
        tmp = specs(i).lower;
        specs(i).lower = specs(i).upper;
        specs(i).upper = tmp;
    end
    if ~any(specs(i).type == ["real", "integer"])
        error('Unsupported param type for %s: %s', specs(i).name, specs(i).type);
    end
end
end

function name = normalize_param_name(name)
name = string(strtrim(char(string(name))));
if startsWith(name, "public_vars.") || startsWith(name, "read_only_vars.") || startsWith(name, "map.")
    return;
end
name = "public_vars." + name;
end

function population = initialize_population(pop_size, param_specs)
dim = numel(param_specs);
population = zeros(pop_size, dim);
for j = 1:dim
    lo = param_specs(j).lower;
    hi = param_specs(j).upper;
    population(:, j) = lo + (hi - lo) * rand(pop_size, 1);
    if param_specs(j).type == "integer"
        population(:, j) = round(population(:, j));
    end
end

for j = 1:dim
    if isfield(param_specs(j), 'initial') && ~isempty(param_specs(j).initial)
        population(1, j) = cast_param_value(param_specs(j).initial, param_specs(j));
    else
        population(1, j) = cast_param_value((param_specs(j).lower + param_specs(j).upper) / 2, param_specs(j));
    end
end
population = clamp_population(population, param_specs);
end

function [evals, population] = sort_population(evals, population)
[~, order] = sort([evals.score], 'ascend');
evals = evals(order);
population = population(order, :);
end

function population = breed_next_population(population, evals, param_specs, cfg)
next_population = zeros(size(population));
elite_count = cfg.elite_count;
next_population(1:elite_count, :) = population(1:elite_count, :);

fill_idx = elite_count + 1;
while fill_idx <= size(population, 1)
    p1 = tournament_select(population, evals, cfg.tournament_size);
    p2 = tournament_select(population, evals, cfg.tournament_size);
    child = p1;
    if rand() < cfg.crossover_rate
        alpha = rand(1, size(population, 2));
        child = alpha .* p1 + (1 - alpha) .* p2;
    end
    child = mutate_child(child, param_specs, cfg);
    next_population(fill_idx, :) = child;
    fill_idx = fill_idx + 1;
end

population = clamp_population(next_population, param_specs);
end

function winner = tournament_select(population, evals, tournament_size)
idx = randperm(size(population, 1), tournament_size);
[~, local_best] = min([evals(idx).score]);
winner = population(idx(local_best), :);
end

function child = mutate_child(child, param_specs, cfg)
for j = 1:numel(param_specs)
    if rand() > cfg.mutation_rate
        continue;
    end
    width = param_specs(j).upper - param_specs(j).lower;
    sigma = max(width * cfg.mutation_sigma_fraction, 1e-6);
    if rand() < 0.35
        child(j) = param_specs(j).lower + width * rand();
    else
        child(j) = child(j) + sigma * randn();
    end
    child(j) = cast_param_value(child(j), param_specs(j));
end
end

function population = clamp_population(population, param_specs)
for j = 1:numel(param_specs)
    population(:, j) = max(population(:, j), param_specs(j).lower);
    population(:, j) = min(population(:, j), param_specs(j).upper);
    if param_specs(j).type == "integer"
        population(:, j) = round(population(:, j));
    end
end
end

function eval_result = evaluate_candidate(values, param_specs, cfg, cache, run_idx, generation_idx, individual_idx, project_dir)
values = clamp_population(values, param_specs);
key = candidate_cache_key(values, param_specs);
if isKey(cache, key)
    eval_result = cache(key);
    eval_result.run_idx = run_idx;
    eval_result.generation_idx = generation_idx;
    eval_result.individual_idx = individual_idx;
    eval_result.cached = true;
    return;
end

session_label = sprintf('%s_r%02d_g%03d_i%03d_%06d', ...
    sanitize_label(cfg.session_label), run_idx, generation_idx, individual_idx, randi(999999));
overrides = build_override_tree(param_specs, values);

global PROJECT_TUNING_OVERRIDES;
PROJECT_TUNING_OVERRIDES = overrides;
cleanup = onCleanup(@() clear_global_tuning_overrides());

setenv('PROJECT_DEBUG_MAP_LIST', char(join(string(cfg.map_list), ',')));
setenv('PROJECT_DEBUG_REPEAT_LIST', char(strjoin(cellstr(compose('%d', cfg.repeat_list)), ',')));
setenv('PROJECT_DEBUG_VIEW', '0');
setenv('PROJECT_DEBUG_TIMEOUT', sprintf('%d', cfg.timeout_steps));
setenv('PROJECT_CONTROLLER_MODE', char(string(cfg.controller_mode)));
setenv('PROJECT_SMOOTHING_MODE', char(string(cfg.path_smoothing_mode)));
setenv('PROJECT_LOCALIZATION_MODE', char(string(cfg.localization_mode)));
setenv('PROJECT_DEBUG_SAVE_OUTPUTS', logical_to_env_local(cfg.save_outputs));
setenv('PROJECT_DEBUG_CONCURRENT_RUNS', sprintf('%d', cfg.case_concurrency));
setenv('PROJECT_SESSION_LABEL', session_label);
setenv('PROJECT_DEBUG_WARM', '1');
setenv('PROJECT_CASES_FILE', cfg.cases_file);

project_tuning_debug;

summary_path = find_debug_summary_path(project_dir, session_label);

if ~isfile(summary_path)
    error('Missing candidate summary: %s', summary_path);
end

summary_tbl = readtable(summary_path, 'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');
score_components = compute_case_score_components(summary_tbl, cfg);

eval_result = empty_eval_result();
eval_result.run_idx = run_idx;
eval_result.generation_idx = generation_idx;
eval_result.individual_idx = individual_idx;
eval_result.cached = false;
eval_result.values = values;
eval_result.param_struct = flatten_param_values(param_specs, values);
eval_result.summary_tbl = summary_tbl;
eval_result.success_count = sum(summary_tbl.status == "goal");
eval_result.case_count = height(summary_tbl);
eval_result.success_rate = eval_result.success_count / max(eval_result.case_count, 1);
eval_result.mean_goal_error_m = mean(summary_tbl.goal_error_m, 'omitnan');
eval_result.max_goal_error_m = max(summary_tbl.goal_error_m, [], 'omitnan');
eval_result.mean_runtime_s = mean(summary_tbl.runtime_s, 'omitnan');
eval_result.mean_steps = mean(summary_tbl.steps, 'omitnan');
eval_result.mean_travelled_m = mean(summary_tbl.travelled_m, 'omitnan');
eval_result.timeout_count = sum(summary_tbl.status == "timeout");
eval_result.fail_count = sum(summary_tbl.status ~= "goal");
eval_result.status_signature = join(groupcounts_string(summary_tbl.status), '; ');
eval_result.score = sum(score_components);

cache(key) = eval_result;
delete(cleanup);
clear_global_tuning_overrides();
end

function score_components = compute_case_score_components(summary_tbl, ~)
status_penalty = zeros(height(summary_tbl), 1);
for i = 1:height(summary_tbl)
    switch char(summary_tbl.status(i))
        case 'goal'
            status_penalty(i) = 0;
        case 'timeout'
            status_penalty(i) = 3500;
        otherwise
            status_penalty(i) = 5000;
    end
end

steps_term = 0.35 * double(summary_tbl.steps);
goal_term = 900 * double(summary_tbl.goal_error_m);
runtime_term = 30 * double(summary_tbl.runtime_s);
travel_term = 12 * double(summary_tbl.travelled_m);
score_components = status_penalty + steps_term + goal_term + runtime_term + travel_term;
end

function param_struct = flatten_param_values(param_specs, values)
param_struct = struct();
for i = 1:numel(param_specs)
    safe_name = regexprep(char(param_specs(i).name), '[^A-Za-z0-9]+', '_');
    param_struct.(safe_name) = values(i);
end
end

function rows = build_eval_rows(run_idx, generation_idx, evals, param_specs)
rows = repmat(empty_eval_row(param_specs), numel(evals), 1);
for i = 1:numel(evals)
    rows(i).run_idx = run_idx;
    rows(i).generation_idx = generation_idx;
    rows(i).individual_idx = i;
    rows(i).score = evals(i).score;
    rows(i).success_rate = evals(i).success_rate;
    rows(i).success_count = evals(i).success_count;
    rows(i).case_count = evals(i).case_count;
    rows(i).mean_goal_error_m = evals(i).mean_goal_error_m;
    rows(i).max_goal_error_m = evals(i).max_goal_error_m;
    rows(i).mean_runtime_s = evals(i).mean_runtime_s;
    rows(i).mean_steps = evals(i).mean_steps;
    rows(i).mean_travelled_m = evals(i).mean_travelled_m;
    rows(i).timeout_count = evals(i).timeout_count;
    rows(i).fail_count = evals(i).fail_count;
    rows(i).cached = evals(i).cached;
    rows(i).status_signature = char(evals(i).status_signature);
    for j = 1:numel(param_specs)
        safe_name = regexprep(char(param_specs(j).name), '[^A-Za-z0-9]+', '_');
        rows(i).(safe_name) = evals(i).values(j);
    end
end
end

function row = build_generation_row(run_idx, generation_idx, evals)
scores = [evals.score];
row = struct( ...
    'run_idx', run_idx, ...
    'generation_idx', generation_idx, ...
    'best_score', evals(1).score, ...
    'mean_score', mean(scores), ...
    'median_score', median(scores), ...
    'worst_score', max(scores), ...
    'best_success_rate', evals(1).success_rate, ...
    'best_mean_goal_error_m', evals(1).mean_goal_error_m, ...
    'best_mean_runtime_s', evals(1).mean_runtime_s, ...
    'best_mean_steps', evals(1).mean_steps);
end

function row = build_run_row(run_idx, best_eval, param_specs)
row = struct( ...
    'run_idx', run_idx, ...
    'best_score', best_eval.score, ...
    'success_rate', best_eval.success_rate, ...
    'success_count', best_eval.success_count, ...
    'case_count', best_eval.case_count, ...
    'mean_goal_error_m', best_eval.mean_goal_error_m, ...
    'max_goal_error_m', best_eval.max_goal_error_m, ...
    'mean_runtime_s', best_eval.mean_runtime_s, ...
    'mean_steps', best_eval.mean_steps, ...
    'mean_travelled_m', best_eval.mean_travelled_m, ...
    'status_signature', char(best_eval.status_signature));
for j = 1:numel(param_specs)
    safe_name = regexprep(char(param_specs(j).name), '[^A-Za-z0-9]+', '_');
    row.(safe_name) = best_eval.values(j);
end
end

function write_ga_report(report_path, cfg, param_specs, run_tbl, generation_tbl, best_eval)
fid = fopen(report_path, 'w');
if fid == -1
    error('Could not open %s for writing.', report_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Project tuning GA\n\n');
fprintf(fid, '- maps: `%s`\n', strjoin(cellstr(cfg.map_list), ', '));
fprintf(fid, '- repeats: `%s`\n', strjoin(compose('%d', cfg.repeat_list), ', '));
fprintf(fid, '- timeout: `%d`\n', cfg.timeout_steps);
fprintf(fid, '- runs: `%d`\n', cfg.num_runs);
fprintf(fid, '- population: `%d`\n', cfg.population_size);
fprintf(fid, '- generations: `%d`\n', cfg.generations);
fprintf(fid, '- best score: `%.3f`\n', best_eval.score);
fprintf(fid, '- best success rate: `%.3f`\n', best_eval.success_rate);
fprintf(fid, '- best mean goal error: `%.3f m`\n', best_eval.mean_goal_error_m);
fprintf(fid, '- best mean runtime: `%.3f s`\n\n', best_eval.mean_runtime_s);

fprintf(fid, '## Tuned parameters\n\n');
for i = 1:numel(param_specs)
    fprintf(fid, '- `%s`: `[%.6g, %.6g]`, best=`%.6g`\n', ...
        char(param_specs(i).name), param_specs(i).lower, param_specs(i).upper, best_eval.values(i));
end

fprintf(fid, '\n## Run comparison\n\n');
for i = 1:height(run_tbl)
    fprintf(fid, '- run `%d`: score=`%.3f`, success=`%.3f`, goalErr=`%.3f m`, runtime=`%.3f s`\n', ...
        run_tbl.run_idx(i), run_tbl.best_score(i), run_tbl.success_rate(i), ...
        run_tbl.mean_goal_error_m(i), run_tbl.mean_runtime_s(i));
end

fprintf(fid, '\n## Generation trend\n\n');
for i = 1:height(generation_tbl)
    fprintf(fid, '- run `%d`, gen `%d`: best=`%.3f`, mean=`%.3f`\n', ...
        generation_tbl.run_idx(i), generation_tbl.generation_idx(i), ...
        generation_tbl.best_score(i), generation_tbl.mean_score(i));
end

fprintf(fid, '\n## Best candidate case stats\n\n');
for i = 1:height(best_eval.summary_tbl)
    row = best_eval.summary_tbl(i, :);
    fprintf(fid, '- `%s` run `%d`: status=`%s`, steps=`%d`, goalErr=`%.3f m`, runtime=`%.3f s`, travelled=`%.3f m`\n', ...
        row.map_name, row.repeat_id, row.status, row.steps, row.goal_error_m, row.runtime_s, row.travelled_m);
end
end

function overrides = build_override_tree(param_specs, values)
overrides = struct();
for i = 1:numel(param_specs)
    path_parts = split(char(param_specs(i).name), '.');
    value = cast_param_value(values(i), param_specs(i));
    overrides = nested_assign(overrides, path_parts, value);
end
end

function s = nested_assign(s, path_parts, value)
head = matlab.lang.makeValidName(path_parts{1});
if numel(path_parts) == 1
    s.(head) = value;
    return;
end
if ~isfield(s, head) || ~isstruct(s.(head))
    s.(head) = struct();
end
s.(head) = nested_assign(s.(head), path_parts(2:end), value);
end

function out = cast_param_value(value, spec)
value = max(value, spec.lower);
value = min(value, spec.upper);
if spec.type == "integer"
    out = round(value);
else
    out = double(value);
end
end

function key = candidate_cache_key(values, param_specs)
parts = cell(1, numel(values));
for i = 1:numel(values)
    if param_specs(i).type == "integer"
        parts{i} = sprintf('%d', round(values(i)));
    else
        parts{i} = sprintf('%.8f', values(i));
    end
end
key = strjoin(parts, '|');
end

function summary_path = find_debug_summary_path(project_dir, session_label)
summary_path = fullfile(project_dir, 'RunSessions', sprintf('debug_%s', session_label), 'project_tuning_debug_runs.csv');
if isfile(summary_path)
    return;
end

matches = dir(fullfile(project_dir, 'RunSessions', sprintf('debug_%s*', session_label)));
matches = matches([matches.isdir]);
if isempty(matches)
    return;
end

[~, order] = sort([matches.datenum], 'descend');
for i = 1:numel(order)
    candidate = fullfile(matches(order(i)).folder, matches(order(i)).name, 'project_tuning_debug_runs.csv');
    if isfile(candidate)
        summary_path = candidate;
        return;
    end
end
end

function session_dir = create_tuning_session_dir(sessions_root, prefix, session_label)
if ~exist(sessions_root, 'dir')
    mkdir(sessions_root);
end
base = fullfile(sessions_root, sprintf('%s_%s', prefix, sanitize_label(session_label)));
session_dir = base;
suffix = 1;
while exist(session_dir, 'dir')
    session_dir = sprintf('%s_%02d', base, suffix);
    suffix = suffix + 1;
end
mkdir(session_dir);
end

function txt = sanitize_label(txt)
txt = regexprep(char(string(txt)), '[^A-Za-z0-9._-]', '_');
end

function project_root = locate_project_root(project_dir)
project_root = project_dir;
while ~isfile(fullfile(project_root, 'main.m'))
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        error('Could not locate project root from %s', project_dir);
    end
    project_root = parent;
end
end

function cfg = set_default(cfg, field_name, value)
if ~isfield(cfg, field_name) || isempty(cfg.(field_name))
    cfg.(field_name) = value;
end
end

function values = normalize_text_list(raw)
if isstring(raw)
    values = raw(:);
    return;
end
if ischar(raw)
    values = string({raw});
    return;
end
if iscell(raw)
    values = string(raw(:));
    return;
end
values = string(raw(:));
end

function out = logical_to_env_local(value)
if value
    out = '1';
else
    out = '0';
end
end

function clear_global_tuning_overrides()
global PROJECT_TUNING_OVERRIDES;
PROJECT_TUNING_OVERRIDES = [];
end

function groups = groupcounts_string(statuses)
[counts, cats] = groupcounts(categorical(statuses));
[counts, order] = sort(counts, 'descend');
cats = cats(order);
groups = strings(1, numel(cats));
for i = 1:numel(cats)
    groups(i) = sprintf('%s:%d', string(cats(i)), counts(i));
end
end

function result = empty_eval_result()
result = struct( ...
    'run_idx', 0, ...
    'generation_idx', 0, ...
    'individual_idx', 0, ...
    'cached', false, ...
    'score', inf, ...
    'success_rate', 0, ...
    'success_count', 0, ...
    'case_count', 0, ...
    'mean_goal_error_m', inf, ...
    'max_goal_error_m', inf, ...
    'mean_runtime_s', inf, ...
    'mean_steps', inf, ...
    'mean_travelled_m', inf, ...
    'timeout_count', 0, ...
    'fail_count', inf, ...
    'status_signature', "", ...
    'values', [], ...
    'param_struct', struct(), ...
    'summary_tbl', table());
end

function row = empty_eval_row(param_specs)
row = struct( ...
    'run_idx', 0, ...
    'generation_idx', 0, ...
    'individual_idx', 0, ...
    'score', inf, ...
    'success_rate', 0, ...
    'success_count', 0, ...
    'case_count', 0, ...
    'mean_goal_error_m', inf, ...
    'max_goal_error_m', inf, ...
    'mean_runtime_s', inf, ...
    'mean_steps', inf, ...
    'mean_travelled_m', inf, ...
    'timeout_count', 0, ...
    'fail_count', inf, ...
    'cached', false, ...
    'status_signature', '');
for i = 1:numel(param_specs)
    safe_name = regexprep(char(param_specs(i).name), '[^A-Za-z0-9]+', '_');
    row.(safe_name) = nan;
end
end
