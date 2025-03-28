function G_slos_star = minimum_spanning_tree(weights)
    % MINIMUM_SPANNING_TREE: Computes the MST of a weighted graph using Prim's algorithm
    % weights -> NxN matrix representing edge weights (Inf for no connection)
    % Returns:
    %   G_slos_star -> NxN adjacency matrix of the MST
    
    N = size(weights, 1); % Number of robots
    G_slos_star = zeros(N); % Initialize MST adjacency matrix

    % Convert weights into MATLAB's graph representation
    G = graph(weights); 

    % Compute the Minimum Spanning Tree (MST) using Prim's algorithm
    T = minspantree(G, 'Method', 'sparse'); 

    % Extract edges from the MST
    for i = 1:numedges(T)
        edge = T.Edges.EndNodes(i, :);
        G_slos_star(edge(1), edge(2)) = 1;
        G_slos_star(edge(2), edge(1)) = 1; % Undirected graph
    end
end
