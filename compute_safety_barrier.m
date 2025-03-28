function u_sbc = compute_safety_barrier(robot_positions, R_s, obstacles, R_obs, gamma_s)
    % COMPUTE_SAFETY_BARRIER: Ensures robots stay collision-free and avoid obstacles
    % robot_positions -> Nx2 matrix of robot positions
    % R_s -> Minimum inter-robot safe distance
    % obstacles -> List of obstacles [x, y, width, height]
    % R_obs -> Minimum robot-obstacle safe distance
    % gamma_s -> CBF safety parameter

    N = size(robot_positions, 1);
    u_sbc = zeros(N, 2); % Initialize SBC control inputs

    % Compute repulsion forces for safety
    for i = 1:N
        for j = i+1:N
            d_ij = norm(robot_positions(i, :) - robot_positions(j, :));
            if d_ij < R_s
                grad_h = (robot_positions(i, :) - robot_positions(j, :)) / d_ij;
                u_sbc(i, :) = u_sbc(i, :) + gamma_s * (1/d_ij - 1/R_s) * grad_h;
            end
        end
    end

    % Compute repulsion forces for obstacle avoidance
    for i = 1:N
        for o = 1:size(obstacles, 1)
            obs_center = obstacles(o, 1:2) + obstacles(o, 3:4) / 2;
            d_io = norm(robot_positions(i, :) - obs_center);
            if d_io < R_obs
                grad_h = (robot_positions(i, :) - obs_center) / d_io;
                u_sbc(i, :) = u_sbc(i, :) + gamma_s * (1/d_io - 1/R_obs) * grad_h;
            end
        end
    end
end
