function [path] = dijkstra(read_only_vars, public_vars)
%DIJKSTRA Grid-based shortest path planning (uniform-cost search).

[path, ~] = grid_planner_core(read_only_vars, public_vars, 'dijkstra');

end
