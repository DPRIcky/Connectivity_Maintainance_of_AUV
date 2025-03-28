function w_connectivity = compute_connectivity_weight(robot_positions, i, j, nominal_control)
    % COMPUTE_CONNECTIVITY_WEIGHT: Calculates weight for connectivity constraints
    d_ij = norm(robot_positions(i, :) - robot_positions(j, :));
    w_connectivity = d_ij + norm(nominal_control(i, :) - nominal_control(j, :));
end

function w_los = compute_los_weight(robot_positions, i, j, obstacles, nominal_control)
    % COMPUTE_LOS_WEIGHT: Calculates weight for LOS constraints
    midpoint = (robot_positions(i, :) + robot_positions(j, :)) / 2;
    major_axis_length = norm(robot_positions(i, :) - robot_positions(j, :));
    
    % Determine LOS violation level
    if is_obstacle_blocking(midpoint, major_axis_length, obstacles)
        w_los = 100; % High weight if LOS is blocked
    else
        w_los = norm(nominal_control(i, :) - nominal_control(j, :));
    end
end
