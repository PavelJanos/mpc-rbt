function [new_mu, new_sigma] = kf_correct(mu, sigma, z, kf)
%KF_CORRECT Linear Kalman correction step (GNSS measurement update).

[new_mu, new_sigma] = kf_measure(mu, sigma, z, kf);

end
