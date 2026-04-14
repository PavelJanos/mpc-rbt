function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% Task 5 (motion uncertainty): open-loop control without sensor feedback.
% Sequence tuned for indoor_1 from start [1,1,pi/2] toward goal [9,9].
% Command format: [vR, vL].

k = read_only_vars.counter;

if k <= 136
    public_vars.motion_vector = [0.5, 0.5];      % north
elseif k <= 144
    public_vars.motion_vector = [-0.2, 0.2];     % turn right (to east)
elseif k <= 226
    public_vars.motion_vector = [0.5, 0.5];      % east
elseif k <= 234
    public_vars.motion_vector = [-0.2, 0.2];     % turn right (to south)
elseif k <= 357
    public_vars.motion_vector = [0.5, 0.5];      % south
elseif k <= 364
    public_vars.motion_vector = [0.2, -0.2];     % turn left (to east)
elseif k <= 434
    public_vars.motion_vector = [0.5, 0.5];      % east
elseif k <= 442
    public_vars.motion_vector = [0.2, -0.2];     % turn left (to north)
elseif k <= 587
    public_vars.motion_vector = [0.5, 0.5];      % north to goal
else
    public_vars.motion_vector = [0, 0];
end

% Save one screenshot for Task2/Task5 near the end of run.
persistent screenshot_saved;
if isempty(screenshot_saved)
    screenshot_saved = false;
end
if ~screenshot_saved && k >= 590
    screenshot_saved = true;
    try
        current_path = fileparts(mfilename('fullpath'));
        while ~exist(fullfile(current_path, 'main.m'), 'file')
            parent = fileparts(current_path);
            if strcmp(parent, current_path)
                break;
            end
            current_path = parent;
        end
        out_dir = fullfile(current_path, 'algorithms', 'tasks', 'Task2', 'Task5');
        if ~isfolder(out_dir)
            mkdir(out_dir);
        end
        saveas(gcf, fullfile(out_dir, 'task5_open_loop_success.png'));
    catch
        % Optional screenshot export; ignore failures.
    end
end


end
