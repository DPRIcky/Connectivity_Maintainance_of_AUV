function is_blocked = is_obstacle_blocking(midpoint, axis_length, obstacles)
    % IS_OBSTACLE_BLOCKING: Determines if any obstacle blocks the LOS path
    is_blocked = false;

    for o = 1:size(obstacles, 1)
        obs_center = obstacles(o, 1:2) + obstacles(o, 3:4) / 2;
        obs_dist = norm(midpoint - obs_center);
        
        % Check if obstacle is within the ellipsoid approximation
        if obs_dist < axis_length / 2
            is_blocked = true;
            return;
        end
    end
end
