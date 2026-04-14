function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));

origin = pose(1:2);
theta = pose(3);

for i = 1:length(lidar_config)
    ray_dir = wrap_to_pi_local(theta + lidar_config(i));
    intersections = ray_cast(origin, map.walls, ray_dir);

    if isempty(intersections)
        measurement(i) = inf;
        continue;
    end

    % intersections expected as Nx2 points; take nearest intersection.
    delta = intersections - origin;
    d2 = sum(delta.^2, 2);
    measurement(i) = sqrt(min(d2));
end

end

function a = wrap_to_pi_local(a)
a = mod(a + pi, 2 * pi) - pi;
end

