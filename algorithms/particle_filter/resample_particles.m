function [new_particles] = resample_particles(particles, weights, method)
%RESAMPLE_PARTICLES Resample particle set according to particle weights.
% Supported methods: 'multinomial', 'systematic', 'stratified', 'residual'

if nargin < 3 || isempty(method)
    method = 'systematic';
end

N = size(particles, 1);
if N == 0
    new_particles = particles;
    return;
end

w = weights(:);
if numel(w) ~= N || any(~isfinite(w)) || sum(w) <= 0
    w = ones(N, 1) / N;
else
    w = w / sum(w);
end

switch lower(method)
    case 'multinomial'
        idx = multinomial_resample_idx(w, N);
    case 'systematic'
        idx = systematic_resample_idx(w, N);
    case 'stratified'
        idx = stratified_resample_idx(w, N);
    case 'residual'
        idx = residual_resample_idx(w, N);
    otherwise
        idx = systematic_resample_idx(w, N);
end

new_particles = particles(idx, :);

end

function idx = multinomial_resample_idx(w, N)
cdf = cumsum(w);
u = rand(N, 1);
idx = arrayfun(@(x) find(cdf >= x, 1, 'first'), u);
end

function idx = systematic_resample_idx(w, N)
cdf = cumsum(w);
u0 = rand() / N;
u = u0 + (0:(N - 1))' / N;
idx = zeros(N, 1);
j = 1;
for i = 1:N
    while u(i) > cdf(j)
        j = j + 1;
    end
    idx(i) = j;
end
end

function idx = stratified_resample_idx(w, N)
cdf = cumsum(w);
u = ((0:(N - 1))' + rand(N, 1)) / N;
idx = zeros(N, 1);
j = 1;
for i = 1:N
    while u(i) > cdf(j)
        j = j + 1;
    end
    idx(i) = j;
end
end

function idx = residual_resample_idx(w, N)
det_count = floor(N * w);
idx = repelem((1:N)', det_count);

R = N - numel(idx);
if R > 0
    res_w = N * w - det_count;
    s = sum(res_w);
    if s <= 0
        tail = randi(N, R, 1);
    else
        res_w = res_w / s;
        tail = multinomial_resample_idx(res_w, R);
    end
    idx = [idx; tail];
end

% Shuffle to avoid deterministic ordering artifacts.
idx = idx(randperm(N));
end

