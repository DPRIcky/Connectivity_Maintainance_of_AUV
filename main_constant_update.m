clear; clc; close all;

% Define workspace size
workspace_size = [10, 10];

% Define task areas (center positions and spread range)
task_areas = struct(...
    'Task1', [8, 2, 1.5], ...
    'Task2', [2, 2, 1.5], ...
    'Task3', [2, 8, 1.5], ...
    'Task4', [8, 8, 1.5]);

% Define obstacles as rectangles [x, y, width, height]
obstacles = [2, 6, 2, 0.25; 6, 4, 4, 0.25; 4, 0, 0.25, 4];

% Parameters
N = 100; % Number of robots
R_c = 5.0; % Strict connectivity range constraint
R_s = 1.5; R_obs = 1.0;
gamma_s = 1.5; gamma_c = 1.0; gamma_los = 1.0;
umax = 1.0; max_iterations = 1500; step_size = 0.5;
relay_threshold = 0.5; % 🚀 **Max LOS Edge Length before forcing relay**
circle_radius = 1.0; % **Circle formation radius**
task_keys = fieldnames(task_areas);
task_colors = {'b', 'r', 'g', 'm'};

% Start all robots from one task
start_task = task_keys{randi(4)};
start_location = task_areas.(start_task)(1:2);
spread = task_areas.(start_task)(3);

robot_positions = zeros(N, 2);
robot_tasks = strings(N, 1);
robot_goals = zeros(N, 2);

% Assign robots initial positions and tasks
for i = 1:N
    robot_positions(i, :) = start_location + (rand(1, 2) - 0.5) * spread;
    task_index = randi(4);
    assigned_task = task_keys{task_index};
    robot_tasks(i) = assigned_task;
    robot_goals(i, :) = task_areas.(assigned_task)(1:2);
end

% Visualization setup
figure; hold on; axis equal;
xlim([0 workspace_size(1)]); ylim([0 workspace_size(2)]);
title('MLCCST with Dynamic Relay Allocation & Circular Goal Formation');

% Plot task areas
for t = 1:4
    center = task_areas.(task_keys{t})(1:2);
    pos = [center(1)-0.1, center(2)-0.1, 0.2, 0.2];
    rectangle('Position', pos, 'Curvature', [1 1], ...
              'FaceColor', task_colors{t}, 'EdgeColor', 'k', ...
              'LineWidth', 1, 'FaceAlpha', 0.3);
end

% Plot obstacles
for i = 1:size(obstacles, 1)
    rectangle('Position', obstacles(i, :), 'FaceColor', [0 0 0], 'EdgeColor', 'k');
end

% Initialize robot visuals
robot_handles = gobjects(N, 1);
los_handles = gobjects(N, N);
for i = 1:N
    task_index = find(strcmp(task_keys, robot_tasks(i)));
    robot_handles(i) = scatter(robot_positions(i, 1), robot_positions(i, 2), ...
                               20, task_colors{task_index}, 'filled');
end

% MLCCST Simulation
for t = 1:max_iterations
    nominal_control = zeros(N, 2);

    % Compute global LOS graph
    G_los = build_los_graph(robot_positions, R_c, obstacles);

    % Compute MLCCST for minimal LOS constraints
    G_slos_star = compute_mlccst(G_los, robot_positions, obstacles, nominal_control);

    % Assign relays dynamically based on LOS threshold
    relay_nodes = false(N, 1);
    goal_reached = false(N, 1);

    % **Step 1: Move Towards Goal or Become Relay**
    for i = 1:N
        direction = robot_goals(i, :) - robot_positions(i, :);

        % 🚀 **Ensure LOS Edge Length doesn't exceed threshold**
        neighbors = find(G_slos_star(i, :));
        for j = neighbors
            if norm(robot_positions(i, :) - robot_positions(j, :)) > relay_threshold
                relay_nodes(i) = true; % Make this a relay
                direction = [0, 0]; % Stop movement
                break; % Only need one relay per edge violation
            end
        end

        % 🚀 **If close to the goal, start circular formation**
        if norm(direction) < 1.5
            goal_reached(i) = true;
            theta = (i / N) * 2 * pi; % Unique angle for each robot
            circle_position = robot_goals(i, :) + [circle_radius * cos(theta), circle_radius * sin(theta)];
            direction = circle_position - robot_positions(i, :);
        end

        % 🚀 **Ensure relays reposition to maintain tree connectivity**
        if relay_nodes(i)
            nearest_neighbors = find(G_slos_star(i, :));
            if ~isempty(nearest_neighbors)
                avg_position = mean(robot_positions(nearest_neighbors, :), 1);
                direction = avg_position - robot_positions(i, :);
            end
        end

        % 🚀 **Keep all robots within strict LOS range**
        if norm(direction) > R_c
            direction = direction / norm(direction) * R_c;
        end

        if norm(direction) > 0
            nominal_control(i, :) = direction / norm(direction) * step_size;
        else
            nominal_control(i, :) = [0, 0];
        end
    end

    % Apply CBF-based constraints
    u_sbc = compute_safety_barrier(robot_positions, R_s, obstacles, R_obs, gamma_s);
    u_cbc = compute_connectivity_barrier(robot_positions, G_slos_star, R_c, gamma_c);
    u_los_cbc = compute_los_cbc(robot_positions, G_slos_star, obstacles, R_c, gamma_los);
    u_total = solve_qp(nominal_control, u_sbc, u_cbc, u_los_cbc, umax);

    % Update positions
    robot_positions = robot_positions + u_total * step_size;

    % Update visuals
    for i = 1:N
        set(robot_handles(i), 'XData', robot_positions(i, 1), 'YData', robot_positions(i, 2));
        if relay_nodes(i)
            set(robot_handles(i), 'Marker', 'o', 'MarkerEdgeColor', 'k', 'SizeData', 30);
        end
    end

    delete(los_handles);
    for i = 1:N
        for j = i+1:N
            if G_slos_star(i, j) == 1
                los_handles(i, j) = plot([robot_positions(i,1), robot_positions(j,1)], ...
                                         [robot_positions(i,2), robot_positions(j,2)], ...
                                         'k--', 'LineWidth', 0.5);
            end
        end
    end

    drawnow limitrate;

    goal_distances = vecnorm(robot_positions - robot_goals, 2, 2);
    if all(goal_distances < 0.3)
        disp('All robots stabilized under MLCCST constraints with Circular Formation.');
        break;
    end
end
