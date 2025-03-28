function plot_workspace(robot_positions, obstacles, G_slos_star, task_areas, task_colors, task_keys, robot_tasks)
    % PLOT_WORKSPACE: Visualizes robots, obstacles, task areas, and LOS communication
    % robot_positions -> Nx2 matrix (robot locations)
    % obstacles -> List of obstacles [x, y, width, height]
    % G_slos_star -> LOS communication adjacency matrix
    % task_areas -> Struct with task area positions
    % task_colors -> Cell array of colors for each task
    % task_keys -> Field names of task_areas
    % robot_tasks -> Array containing assigned tasks for each robot

    figure; hold on; axis equal;
    xlim([0 10]); ylim([0 10]);
    title('LOS Communication Network with CBF Constraints');

    % Define task area size
    task_area_size = [0.2, 0.2]; 

    % Plot task areas as rectangles
    for t = 1:4
        task_name = task_keys{t}; 
        center = task_areas.(task_name)(1:2);
        bottom_left = center - task_area_size / 2;
        rectangle('Position', [bottom_left, task_area_size], ...
                  'FaceColor', task_colors{t}, 'EdgeColor', 'k', 'LineWidth', 0.5);
    end

    % % Plot obstacles
    % for i = 1:size(obstacles, 1)
    %     rectangle('Position', obstacles(i, :), 'FaceColor', [0 0 0], 'EdgeColor', 'k');
    % end

    % Plot robots with colors matching their assigned task
    for i = 1:size(robot_positions, 1)
        task_index = find(strcmp(task_keys, robot_tasks(i))); % Match task to color
        scatter(robot_positions(i, 1), robot_positions(i, 2), 20, task_colors{task_index}, 'filled');
    end

    % Draw LOS communication edges
    for i = 1:size(robot_positions, 1)
        for j = i+1:size(robot_positions, 1)
            if G_slos_star(i, j) == 1
                plot([robot_positions(i,1), robot_positions(j,1)], ...
                     [robot_positions(i,2), robot_positions(j,2)], 'r--', 'LineWidth', 0.5);
            end
        end
    end

    % Create a legend with correct colors for task areas
    legend_handles = gobjects(1, 4);
    for t = 1:4
        legend_handles(t) = scatter(nan, nan, 100, task_colors{t}, 'filled');
    end
    legend(legend_handles, {'Task1', 'Task2', 'Task3', 'Task4'}, 'Location', 'northeast');

    grid on;
end
