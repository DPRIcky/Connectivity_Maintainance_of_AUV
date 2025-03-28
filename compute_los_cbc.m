function u_los_cbc = compute_los_cbc(robot_positions, G_los, obstacles, R_c, gamma_los)
    % COMPUTE_LOS_CBC: Ensures LOS connectivity using barrier certificates
    % robot_positions -> Nx2 matrix of robot positions
    % G_los -> LOS adjacency matrix
    % obstacles -> List of obstacles
    % R_c -> Communication range
    % gamma_los -> LOS-CBF parameter

    N = size(robot_positions, 1);
    u_los_cbc = zeros(N, 2); % Initialize LOS CBC control inputs

    for i = 1:N
        for j = i+1:N
            if G_los(i, j) == 1
                % Compute the Minimum Volume Enclosing Ellipsoid (MVEE) approximation
                midpoint = (robot_positions(i, :) + robot_positions(j, :)) / 2;
                major_axis_length = norm(robot_positions(i, :) - robot_positions(j, :));

                % Check if any obstacle is inside the MVEE region
                if is_obstacle_blocking(midpoint, major_axis_length, obstacles)
                    grad_h = (robot_positions(j, :) - robot_positions(i, :)) / norm(robot_positions(i, :) - robot_positions(j, :));
                    u_los_cbc(i, :) = u_los_cbc(i, :) + gamma_los * grad_h;
                end
            end
        end
    end
end
