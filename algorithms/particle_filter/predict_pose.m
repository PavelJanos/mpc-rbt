function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

Ts = read_only_vars.sampling_period;
drive = read_only_vars.agent_drive;

vR = motion_vector(1);
vL = motion_vector(2);

if drive.type == 2
    % Differential drive kinematics.
    v = 0.5 * (vR + vL);
    omega = (vR - vL) / drive.interwheel_dist;

    dx = v * cos(old_pose(3)) * Ts;
    dy = v * sin(old_pose(3)) * Ts;
    dtheta = omega * Ts;
else
    % Fallback for non-differential setup.
    v = 0.5 * (vR + vL);
    dx = v * cos(old_pose(3)) * Ts;
    dy = v * sin(old_pose(3)) * Ts;
    dtheta = 0;
end

% Probabilistic motion model: add zero-mean Gaussian noise.
sigma_xy_base = 0.01;
sigma_theta_base = 0.02;
sigma_xy_gain = 0.05;
sigma_theta_gain = 0.08;

speed_abs = abs(vR) + abs(vL);
sigma_xy = sigma_xy_base + sigma_xy_gain * speed_abs * Ts;
sigma_theta = sigma_theta_base + sigma_theta_gain * abs(vR - vL) * Ts;

noise_x = sigma_xy * randn();
noise_y = sigma_xy * randn();
noise_theta = sigma_theta * randn();

new_pose = old_pose;
new_pose(1) = old_pose(1) + dx + noise_x;
new_pose(2) = old_pose(2) + dy + noise_y;
new_pose(3) = wrap_to_pi_local(old_pose(3) + dtheta + noise_theta);

end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2 * pi) - pi;
end

