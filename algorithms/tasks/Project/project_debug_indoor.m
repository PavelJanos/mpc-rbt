%% Project indoor debug
clear; clc; close all;

setenv('PROJECT_DEBUG_MAP', 'indoor_2');
setenv('PROJECT_DEBUG_REPEAT', '1');
setenv('PROJECT_DEBUG_VIEW', 'true');
setenv('PROJECT_CONTROLLER_MODE', 'pure_pursuit');
setenv('PROJECT_SMOOTHING_MODE', 'iterative');
setenv('PROJECT_LOCALIZATION_MODE', 'fusion');

run(fullfile(fileparts(mfilename('fullpath')), 'project_tuning_debug.m'));
