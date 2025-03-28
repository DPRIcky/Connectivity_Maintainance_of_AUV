function obstacles = define_obstacles()
    % DEFINE_OBSTACLES: Creates a set of static polyhedral obstacles
    % Returns a struct containing obstacle centers and sizes

    % Define obstacle positions and sizes (polyhedral obstacles)
    obstacles(1).center = [3, 4];
    obstacles(1).size = [2, 2];  % Width and height

    obstacles(2).center = [6, 7];
    obstacles(2).size = [1.5, 1.5];

    obstacles(3).center = [5, 2];
    obstacles(3).size = [2, 1];

    % Add more obstacles as needed...
end
