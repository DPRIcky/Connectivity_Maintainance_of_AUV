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
obstacles = [... 
    2, 6, 2, 0.25;  
    6, 4, 4, 0.25;  
    4, 0, 0.25, 4];

% Define parameters
N = 40; % Number of robots
R_c = 4.0; % Communication range
R_s = 1.5; % Safe inter-robot distance
R_obs = 1.0; % Safe robot-obstacle distance
gamma_s = 1.5; % Safety CBF parameter
gamma_c = 1.0; % Connectivity CBF parameter
gamma_los = 1.0; % LOS-CBF parameter

% Assign robots to a task area
[robot_positions, selected_task] = assign_robots_to_task(N, task_areas);

% Define task area names and colors
task_keys = fieldnames(task_areas);
task_colors = {'b', 'r', 'g', 'm'}; % Task1=blue, Task2=red, Task3=green, Task4=magenta

% Assign each robot a task randomly
robot_tasks = strings(N, 1);
for i = 1:N
    robot_tasks(i) = task_keys{randi(4)}; % Randomly assign a task to each robot
end

% Generate nominal control inputs (random for now)
nominal_control = rand(N, 2) * 0.1; % Example task control

% Compute LOS communication graph
G_los = build_los_graph(robot_positions, R_c, obstacles);

% Compute optimal MLCCST graph G_slos*
G_slos_star = compute_mlccst(G_los, robot_positions, obstacles, nominal_control);

% Compute CBF-based safety and connectivity constraints
u_sbc = compute_safety_barrier(robot_positions, R_s, obstacles, R_obs, gamma_s);
u_cbc = compute_connectivity_barrier(robot_positions, G_slos_star, R_c, gamma_c);
u_los_cbc = compute_los_cbc(robot_positions, G_slos_star, obstacles, R_c, gamma_los);

% Solve Quadratic Programming (QP) for minimal deviation from nominal control
umax = 1.0; % Maximum control input
u_total = solve_qp(nominal_control, u_sbc, u_cbc, u_los_cbc, umax);

% Update robot positions
for i = 1:N
    robot_positions(i, :) = robot_positions(i, :) + u_total(i, :) * 0.1; % Small step for stability
end

% Plot workspace
plot_workspace(robot_positions, obstacles, G_slos_star, task_areas, task_colors, task_keys, robot_tasks);

% Plot obstacles
for i = 1:size(obstacles, 1)
    rectangle('Position', obstacles(i, :), 'FaceColor', [0 0 0], 'EdgeColor', 'k');
end