function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

[path, ~] = grid_planner_core(read_only_vars, public_vars, 'astar');

end

