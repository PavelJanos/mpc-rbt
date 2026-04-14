%% Project random benchmark
clear; clc; close all;

setenv('PROJECT_BENCHMARK_MODE', 'full');
setenv('PROJECT_CASE_MODE', 'regenerate');
setenv('PROJECT_VIEW_MODE', 'false');

run(fullfile(fileparts(mfilename('fullpath')), 'project_test_all_maps.m'));
