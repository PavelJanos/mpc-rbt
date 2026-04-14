%% Task 5 helper: tune open-loop command sequence for indoor_1
clear; clc;

project_root = fileparts(mfilename('fullpath'));
while ~exist(fullfile(project_root, 'main.m'), 'file')
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        error('Could not locate project root (main.m).');
    end
    project_root = parent;
end
addpath(fullfile(project_root, 'utils'));
addpath(fullfile(project_root, 'utils', 'render_factories'));
addpath(fullfile(project_root, 'algorithms'));
addpath(fullfile(project_root, 'algorithms', 'motion_control'));
addpath(fullfile(project_root, 'algorithms', 'particle_filter'));
addpath(fullfile(project_root, 'algorithms', 'kalman_filter'));
addpath(fullfile(project_root, 'algorithms', 'path_planning'));

map = load_map(fullfile(project_root, 'maps', 'indoor_1.txt'));
drive = struct('type', 2, 'interwheel_dist', 0.2, 'max_vel', 1);
Ts = 0.1;
goal = map.goal(:)';

best_err = inf;
best = [];

rng(1);
num_trials = 3000;

for t = 1:num_trials
    % Sequence: N -> E -> S -> E -> N with in-place quarter turns
    n1 = randi([110, 200]); % forward north
    n2 = randi([70, 170]);  % forward east
    n3 = randi([60, 170]);  % forward south
    n4 = randi([20, 120]);  % forward east
    n5 = randi([80, 220]);  % forward north

    r1 = randi([7, 10]); % turn to east  from north
    r2 = randi([7, 10]); % turn to south from east
    r3 = randi([7, 10]); % turn to east  from south
    r4 = randi([7, 10]); % turn to north from east

    cmds = [
        repmat([0.5, 0.5], n1, 1);
        repmat([-0.2, 0.2], r1, 1);
        repmat([0.5, 0.5], n2, 1);
        repmat([-0.2, 0.2], r2, 1);
        repmat([0.5, 0.5], n3, 1);
        repmat([0.2, -0.2], r3, 1);
        repmat([0.5, 0.5], n4, 1);
        repmat([0.2, -0.2], r4, 1);
        repmat([0.5, 0.5], n5, 1)
    ];

    pose = [1, 1, pi/2];
    valid = true;

    for k = 1:size(cmds, 1)
        pose = move_agent(pose, cmds(k, :), drive, Ts);
        p = pose(1:2);

        if p(1) < 0 || p(1) > 10 || p(2) < 0 || p(2) > 10
            valid = false;
            break;
        end

        if is_collision_lines(p, map.walls, 0.05)
            valid = false;
            break;
        end
    end

    if ~valid
        continue;
    end

    err = norm(pose(1:2) - goal);
    if err < best_err
        best_err = err;
        best = [n1 n2 n3 n4 n5 r1 r2 r3 r4 pose];
    end
end

disp('Best candidate: [n1 n2 n3 n4 n5 r1 r2 r3 r4 x y theta]');
disp(best);
disp('Goal error:');
disp(best_err);

function hit = is_collision_lines(point, walls, tol)
hit = false;
for i = 1:size(walls, 1)
    a = walls(i, 1:2);
    b = walls(i, 3:4);
    ab = b - a;
    if all(ab == 0)
        dist = norm(point - a);
    else
        t = dot(point - a, ab) / dot(ab, ab);
        t = max(0, min(1, t));
        proj = a + t * ab;
        dist = norm(point - proj);
    end
    if dist <= tol
        hit = true;
        return;
    end
end
end
