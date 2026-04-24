function [path] = greedy_best_first(read_only_vars, public_vars)
%GREEDY_BEST_FIRST Grid-based best-first path planning.

[path, ~] = grid_planner_core(read_only_vars, public_vars, 'greedy');

end
