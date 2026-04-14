%% Project localization comparison
clear; clc; close all;

project_dir = fileparts(mfilename('fullpath'));
modes = {'fusion', 'pf_only', 'kf_only'};
rows = cell(numel(modes), 4);

setenv('PROJECT_DEBUG_MAP', 'indoor_2');
setenv('PROJECT_DEBUG_REPEAT', '1');
setenv('PROJECT_DEBUG_VIEW', 'false');
setenv('PROJECT_CONTROLLER_MODE', 'pure_pursuit');
setenv('PROJECT_SMOOTHING_MODE', 'iterative');

for i = 1:numel(modes)
    setenv('PROJECT_LOCALIZATION_MODE', modes{i});
    run(fullfile(project_dir, 'project_tuning_debug.m'));
    report_path = fullfile(project_dir, 'project_tuning_debug_indoor_2_run_01_report.md');
    report = string(splitlines(fileread(report_path)));
    status_line = report(startsWith(strtrim(report), "- vysledek:"));
    goal_line = report(startsWith(strtrim(report), "- final goal error:"));
    travel_line = report(startsWith(strtrim(report), "- travelled:"));
    rows(i, :) = {modes{i}, extractAfter(status_line, "`"), extractAfter(goal_line, "`"), extractAfter(travel_line, "`")};
end

tbl = cell2table(rows, 'VariableNames', {'localization_mode', 'status_raw', 'goal_error_raw', 'travelled_raw'});
writetable(tbl, fullfile(project_dir, 'project_compare_localization_modes.csv'));
