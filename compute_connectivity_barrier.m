function u_cbc = compute_connectivity_barrier(robot_positions, G_los, R_c, gamma_c)
    % COMPUTE_CONNECTIVITY_BARRIER: Maintains connectivity within communication range
    % robot_positions -> Nx2 matrix of robot positions
    % G_los -> LOS communication adjacency matrix
    % R_c -> Maximum communication range
    % gamma_c -> Connectivity CBF parameter

    N = size(robot_positions, 1);
    u_cbc = zeros(N, 2); % Initialize CBC control inputs

    % Compute connectivity forces
    for i = 1:N
        for j = i+1:N
            if G_los(i, j) == 1
                d_ij = norm(robot_positions(i, :) - robot_positions(j, :));
                if d_ij > R_c
                    grad_h = (robot_positions(j, :) - robot_positions(i, :)) / d_ij;
                    u_cbc(i, :) = u_cbc(i, :) + gamma_c * (1/d_ij - 1/R_c) * grad_h;
                end
            end
        end
    end
end
