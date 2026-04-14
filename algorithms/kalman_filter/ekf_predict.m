function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

if isempty(mu) || isempty(sigma) || isempty(u)
    new_mu = mu;
    new_sigma = sigma;
    return;
end

vR = u(1);
vL = u(2);

% Differential drive model (interwheel distance from simulator setup).
L = 0.2;
v = 0.5 * (vR + vL);
omega = (vR - vL) / L;
theta = mu(3);
dt = sampling_period;

% Nonlinear state propagation g(x,u).
new_mu = mu;
new_mu(1) = mu(1) + v * cos(theta) * dt;
new_mu(2) = mu(2) + v * sin(theta) * dt;
new_mu(3) = wrap_to_pi_local(mu(3) + omega * dt);

% Jacobian F = dg/dx.
F = [1, 0, -v * sin(theta) * dt;
     0, 1,  v * cos(theta) * dt;
     0, 0,  1];

new_sigma = F * sigma * F' + kf.R;

end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2 * pi) - pi;
end

