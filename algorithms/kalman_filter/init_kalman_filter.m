function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

% Linear measurement model: GNSS observes x,y directly.
public_vars.kf.C = [1 0 0;
                    0 1 0];

% Process noise covariance (state propagation uncertainty).
public_vars.kf.R = diag([0.01, 0.01, 0.01]);

% Measurement noise covariance (GNSS uncertainty).
public_vars.kf.Q = diag([0.25, 0.25]);

% Initialize belief (generic defaults).
if isfield(read_only_vars, 'mocap_pose') && ~isempty(read_only_vars.mocap_pose) && all(isfinite(read_only_vars.mocap_pose))
    public_vars.mu = read_only_vars.mocap_pose(:);
else
    public_vars.mu = [0; 0; 0];
end
public_vars.sigma = diag([1e-4, 1e-4, 1e-4]);
public_vars.kf_enabled = 1;

% Task5/Task4 deployment profile for outdoor_1.
if is_outdoor_1_map(read_only_vars.map)
    % Tuned process covariance from Task5/Task4.
    public_vars.kf.R = diag([0.000570, 0.000850, 0.000950]);

    % GNSS-based initialization from Task5/Task1 when available.
    init_file = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
        'tasks', 'Task5', 'Task1', 'task1_gnss_init_data.mat');
    if isfile(init_file)
        s = load(init_file, 'gnss_mu', 'gnss_sigma');
        if isfield(s, 'gnss_mu') && isfield(s, 'gnss_sigma') ...
                && all(isfinite(s.gnss_mu(:))) && all(isfinite(s.gnss_sigma(:)))
            public_vars.mu = [s.gnss_mu(:); 0];
            public_vars.sigma = blkdiag(s.gnss_sigma, 2.0); % high theta uncertainty
            public_vars.kf.Q = 1.18 * s.gnss_sigma;         % tuned Task5/Task4 scale
        end
    end

    % In deployment mode drive from EKF estimate.
    public_vars.use_estimated_pose_only = true;
    if ~isfield(public_vars, 'controller_mode') || isempty(public_vars.controller_mode)
        public_vars.controller_mode = 'pure_pursuit';
    end
end

end

function tf = is_outdoor_1_map(map)
tf = isequal(size(map.walls), [15, 4]) ...
    && isequal(round(map.goal, 6), [16, 2]) ...
    && isequal(round(map.limits, 6), [0, 0, 20, 15]);
end

