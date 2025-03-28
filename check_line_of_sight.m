function is_los = check_line_of_sight(x1, x2, obstacles)
    % CHECK_LINE_OF_SIGHT: Determines if two points have a clear LOS
    % x1, x2 -> Robot positions [x, y]
    % obstacles -> List of obstacle rectangles [x, y, width, height]

    is_los = true; % Assume LOS is clear

    % Discretize line between x1 and x2 into small segments
    num_points = 20; % Number of points to sample along the line
    lambda = linspace(0, 1, num_points);
    line_points = (1 - lambda') * x1 + lambda' * x2; 

    % Check if any point along the line is inside an obstacle
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        for j = 1:num_points
            if line_points(j,1) >= obs(1) && line_points(j,1) <= obs(1) + obs(3) && ...
               line_points(j,2) >= obs(2) && line_points(j,2) <= obs(2) + obs(4)
                is_los = false;
                return;
            end
        end
    end
end
