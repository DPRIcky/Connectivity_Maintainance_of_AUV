function G_los = build_los_graph(robot_positions, R_c, obstacles)
    % BUILD_LOS_GRAPH: Constructs the Line-of-Sight communication graph
    % robot_positions -> Nx2 matrix of robot positions
    % R_c -> Communication range
    % obstacles -> Obstacle list

    N = size(robot_positions, 1); % Number of robots
    G_los = zeros(N); % Adjacency matrix for LOS graph

    % Check connectivity for all pairs (i, j)
    for i = 1:N
        for j = i+1:N
            d = norm(robot_positions(i, :) - robot_positions(j, :));
            if d <= R_c && check_line_of_sight(robot_positions(i, :), robot_positions(j, :), obstacles)
                G_los(i, j) = 1; % Mark edge (i, j)
                G_los(j, i) = 1; % Symmetric edge
            end
        end
    end
end
