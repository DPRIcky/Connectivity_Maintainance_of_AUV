function w_los = compute_los_weight(robot_positions, i, j, obstacles, nominal_control)
    % COMPUTE_LOS_WEIGHT: Assigns a weight to the edge (i, j) based on LOS constraints
    % robot_positions -> Nx2 matrix of robot positions
    % i, j -> Indices of the robots forming the edge
    % obstacles -> List of obstacles
    % nominal_control -> Nx2 matrix of nominal controls for each robot

    % Compute the midpoint of the communication edge
    midpoint = (robot_positions(i, :) + robot_positions(j, :)) / 2;
    
    % Compute the major axis length of the ellipsoid approximation
    major_axis_length = norm(robot_positions(i, :) - robot_positions(j, :));

    % Determine LOS violation level
    if is_obstacle_blocking(midpoint, major_axis_length, obstacles)
        w_los = 100; % High weight if LOS is blocked
    else
        w_los = norm(nominal_control(i, :) - nominal_control(j, :)); % Use control difference as weight
    end
end
