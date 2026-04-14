function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

% Linear measurement model: GNSS observes x,y directly.
public_vars.kf.C = [1 0 0;
                    0 1 0];

% Process noise covariance (state propagation uncertainty).
public_vars.kf.R = diag([0.01, 0.01, 0.01]);

% Measurement noise covariance (GNSS uncertainty).
public_vars.kf.Q = diag([0.25, 0.25]);

% Initialize belief (no MoCap dependency).
if isfield(read_only_vars, 'gnss_position') && ~isempty(read_only_vars.gnss_position) ...
        && all(isfinite(read_only_vars.gnss_position(1:2)))
    public_vars.mu = [read_only_vars.gnss_position(1:2), 0]';
    public_vars.sigma = diag([0.25, 0.25, 1.0]);
else
    public_vars.mu = [0; 0; 0];
    public_vars.sigma = diag([1.0, 1.0, 2.0]);
end
public_vars.kf_enabled = 1;

end

