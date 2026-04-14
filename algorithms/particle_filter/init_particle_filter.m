function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here

map = read_only_vars.map;
limits = map.limits; % [xmin ymin xmax ymax]

N = 1000;
safe_margin = 0.12;
max_tries = 200000;

particles = zeros(N, 3);
filled = 0;
tries = 0;

while filled < N && tries < max_tries
    tries = tries + 1;
    x = limits(1) + (limits(3) - limits(1)) * rand();
    y = limits(2) + (limits(4) - limits(2)) * rand();
    th = -pi + 2 * pi * rand();

    if is_collision_with_walls([x, y], map.walls, safe_margin)
        continue;
    end

    filled = filled + 1;
    particles(filled, :) = [x, y, th];
end

if filled < N
    particles = particles(1:filled, :);
end

public_vars.particles = particles;
public_vars.particle_weights = ones(size(particles, 1), 1) / max(size(particles, 1), 1);
public_vars.pf_enabled = 1;

end

function tf = is_collision_with_walls(point, walls, tol)
tf = false;
for i = 1:size(walls, 1)
    a = walls(i, 1:2);
    b = walls(i, 3:4);
    ab = b - a;
    if all(abs(ab) < 1e-12)
        d = norm(point - a);
    else
        t = dot(point - a, ab) / dot(ab, ab);
        t = max(0, min(1, t));
        p = a + t * ab;
        d = norm(point - p);
    end
    if d <= tol
        tf = true;
        return;
    end
end
end

