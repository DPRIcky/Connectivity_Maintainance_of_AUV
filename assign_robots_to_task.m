function [robot_positions, selected_task] = assign_robots_to_task(N, task_areas)
    % ASSIGN_ROBOTS_TO_TASK: Assigns all robots to one randomly selected task area.
    % N -> Number of robots
    % task_areas -> Struct containing task area positions and spread range
    % Returns:
    %   robot_positions -> Nx2 matrix containing robot positions
    %   selected_task -> The chosen task area name

    task_keys = fieldnames(task_areas); % Get task names
    selected_task = task_keys{randi(4)}; % Randomly pick one task
    task_center = task_areas.(selected_task)(1:2); % Get task center
    spread_range = task_areas.(selected_task)(3); % Get spread range

    % Assign all robots to this task area with some spacing
    robot_positions = zeros(N, 2);
    for i = 1:N
        theta = rand() * 2 * pi; % Random angle
        r = spread_range * sqrt(rand()); % Random radius (uniform distribution)
        robot_positions(i, :) = task_center + [r * cos(theta), r * sin(theta)];
    end
end
