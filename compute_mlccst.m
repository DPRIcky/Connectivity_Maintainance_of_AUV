function G_slos_star = compute_mlccst(G_los, robot_positions, obstacles, nominal_control)
    % COMPUTE_MLCCST: Finds the optimal spanning subgraph G_slos* in LOS graph
    % G_los -> LOS adjacency matrix
    % robot_positions -> Nx2 matrix of robot positions
    % obstacles -> List of obstacles
    % nominal_control -> Task-related control inputs

    N = size(robot_positions, 1);
    weights = inf(N); % Initialize edge weights

    % Compute edge weights based on LOS constraints
    for i = 1:N
        for j = i+1:N
            if G_los(i, j) == 1
                % Compute edge weights for connectivity and LOS constraints
                w_connectivity = compute_connectivity_weight(robot_positions, i, j, nominal_control);
                w_los = compute_los_weight(robot_positions, i, j, obstacles, nominal_control);

                % Final edge weight
                weights(i, j) = w_connectivity + w_los;
                weights(j, i) = weights(i, j);
            end
        end
    end

    % Compute the Minimum Spanning Tree (MST) to get optimal G_slos*
    G_slos_star = minimum_spanning_tree(weights);
end
